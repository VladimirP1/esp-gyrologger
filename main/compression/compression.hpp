#pragma once

#include "quat.hpp"

#include "ext/rans_byte.h"

#include <cstddef>
#include <deque>
#include <tuple>
#include <optional>

static inline int8_t iq_i16_i8(double update, double scale)
{
    return std::min(std::max(fabs(update) * scale * ((update < 0) ? -1.0 : 1.0), -127.), 127.);
}

static inline double deiq_i16_i8(int8_t update, double scale)
{
    return (abs(update) / scale) * ((update < 0) ? -1 : 1);
}

class QuatEncoder
{
public:
    using byte_range_t = std::tuple<std::deque<int8_t>::iterator, std::deque<int8_t>::iterator>;

    explicit QuatEncoder(double error_limit = 0.04, double gyro_scale = 3000)
        : error_limit(error_limit), gyro_scale(gyro_scale)
    {
        uint16_t gyro_scale_int = gyro_scale / 10;
        emit_byte((gyro_scale_int >> 8) & 0xff);
        emit_byte(gyro_scale_int & 0xff);
        gyro_scale = gyro_scale_int * 10;
    }

    bool encode(quat::quat q)
    {
        int iters = 10;
        while (iters--)
        {
            // encode
            quat::quat q_update = quat::prod(quat::conj(q_state), q);
            quat::vec v_update = quat::to_aa(q_update) - v_state;

            quat::vec v_update_quant =
                quat::map(v_update, [&](double x)
                          { return iq_i16_i8(x, gyro_scale); });

            emit_byte(v_update_quant[0]);
            emit_byte(v_update_quant[1]);
            emit_byte(v_update_quant[2]);

            // update decoder state
            quat::vec v_update_dequant =
                quat::map(v_update_quant, [&](double x)
                          { return deiq_i16_i8(x, gyro_scale); });
            v_state += v_update_dequant;
            q_state = quat::normalize(quat::prod(q_state, quat::from_aa(v_state)));

            double error_deg =
                quat::norm(quat::to_aa(quat::prod(quat::conj(q_state), q))) * 180. / M_PI;

            if (error_deg > error_limit)
            {
                emit_byte(-128);
                continue;
            }
            break;
        }
        return iters >= 0;
    }

    byte_range_t get_all_bytes() { return {bytes.begin(), bytes.end()}; }

    bool has_block(size_t requested_size) { return bytes.size() >= requested_size; }

    byte_range_t get_block(size_t requested_size)
    {
        assert(has_block(requested_size));
        auto end = bytes.begin();
        std::advance(end, requested_size);
        return {bytes.begin(), end};
    }

    void drop_block(size_t requested_size)
    {
        assert(has_block(requested_size));
        for (int i = 0; i < requested_size; ++i)
            bytes.pop_front();
    }

    void reset()
    {
        q_state = {1, 0, 0, 0};
        v_state = {0, 0, 0};
        bytes.clear();
    }

    void clear() { bytes.clear(); }

private:
    double error_limit{};
    double gyro_scale{};

    quat::quat q_state{1, 0, 0, 0};
    quat::vec v_state{0, 0, 0};

    std::deque<int8_t> bytes;

    inline void emit_byte(int8_t x) { bytes.push_back(x); }
};

class QuatDecoder
{
public:
    explicit QuatDecoder(double gyro_scale = 3000) : gyro_scale(gyro_scale) {}

    template <class I>
    bool decode(I data_begin, I data_end,
                const std::function<void(const quat::quat &)> &quat_callback)
    {
        for (auto it = data_begin; it != data_end;)
        {
            int8_t a, b, c;
            // clang-format off
            a = *it; ++it; if (it == data_end) break;
            b = *it; ++it; if (it == data_end) break;
            c = *it;
            // clang-format on
            quat::vec v_update_quant = {a * 1.0, b * 1.0, c * 1.0};

            quat::vec v_dequant =
                quat::map(v_update_quant, [&](double x)
                          { return deiq_i16_i8(x, gyro_scale); });
            v_state += v_dequant;

            q_state = quat::normalize(quat::prod(q_state, quat::from_aa(v_state)));

            if (it != data_end)
            {
                ++it;
            }
            else
            {
                quat_callback(q_state);
                break;
            }

            if (*it == -128)
            {
                ++it;
                continue;
            }
            else
            {
                quat_callback(q_state);
            }
        }

        return true;
    }

private:
    double gyro_scale{};

    quat::quat q_state{1, 0, 0, 0};
    quat::vec v_state{0, 0, 0};
};

constexpr std::initializer_list<double> kVarianceTable = {
    0.015625, 0.03125, 0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0};

class VarianceEstimatorExact
{
public:
    explicit VarianceEstimatorExact() {}
    void update(int8_t x)
    {
        sum_ += x * x;
        count_ += 1;
    }
    double get() const { return sum_ / count_; }
    void reset() { sum_ = count_ = 0; }

private:
    double sum_{};
    double count_{};
};

class LaplaceModel
{
public:
    explicit LaplaceModel(int i_var, int scale) : i_var_(i_var), scale_(scale) {}

    double b() const { return sqrt(var() / 2); }

    double var() const { return *(kVarianceTable.begin() + i_var_); }

    int cdf(int x) const
    {
        if (x <= -128)
            return 0;
        if (x > 128)
            return 1 << scale_;

        double cum{};
        double xs = x - .5;
        if (xs < 0)
        {
            cum = exp(xs / b()) / 2;
        }
        else
        {
            cum = 1 - exp(-xs / b()) / 2;
        }

        return int(cum * ((1 << scale_) - 257)) + (x + 128);
    }

    int icdf(int y) const
    {
        int l{-129}, r{129};
        while (l + 1 != r)
        {
            int mid = (l + r) / 2;
            int mid_cdf = cdf(mid);
            if (mid_cdf <= y && cdf(mid + 1) > y)
            {
                return mid;
            }
            if (mid_cdf <= y)
            {
                l = mid;
            }
            else
            {
                r = mid;
            }
        }
        return r;
    }

    int freq(int x) const { return cdf(x + 1) - cdf(x); }

private:
    int i_var_, scale_;
};

class EntropyCoder
{
public:
    EntropyCoder(int freq_scale_bits) : freq_scale_bits_(freq_scale_bits) {}

    template <class I>
    int encode_block(I data_begin, I data_end, uint8_t *&compressed_end_ptr)
    {
        std::for_each(data_begin, data_end,
                      std::bind(&VarianceEstimatorExact::update, &var_est, std::placeholders::_1));
        auto it = std::lower_bound(kVarianceTable.begin(), kVarianceTable.end(), var_est.get());
        if (it == kVarianceTable.end())
        {
            it = kVarianceTable.begin() + kVarianceTable.size() - 1;
        }
        int i_var = std::distance(kVarianceTable.begin(), it);
        printf("i_var = %d\n", i_var);

        LaplaceModel mdl(i_var, freq_scale_bits_);

        RansState state;
        uint8_t *compressed_ptr = compressed_end_ptr;
        RansEncInit(&state);

        uint8_t checksum = 0;
        while (data_begin != data_end)
        {
            --data_end;
            int start = mdl.cdf(*data_end);
            int freq = mdl.cdf(*data_end + 1) - start;
            RansEncPut(&state, &compressed_ptr, start, freq,
                       freq_scale_bits_);
            checksum += static_cast<uint8_t>(*data_end);
        }
        RansEncFlush(&state, &compressed_ptr);
        *(--compressed_ptr) = i_var | (checksum << 5);

        size_t bytes_put = compressed_end_ptr - compressed_ptr;
        compressed_end_ptr = compressed_ptr;
        var_est.reset();
        return bytes_put;
    }

private:
    VarianceEstimatorExact var_est{};
    int freq_scale_bits_;
};

class EntropyDecoder
{
public:
    explicit EntropyDecoder(int freq_scale_bits, size_t block_size)
        : freq_scale_bits_(freq_scale_bits), block_size_(block_size) {}

    size_t decode_block(uint8_t *compressed_data, const std::function<void(int)> byte_decoded_cb)
    {
        int i_var = compressed_data[0] & 0x1f;
        uint8_t own_checksum = 0;
        uint8_t checksum = compressed_data[0] >> 5;

        LaplaceModel mdl(i_var, freq_scale_bits_);

        RansState state;
        uint8_t *decode_ptr = compressed_data + 1;
        RansDecInit(&state, &decode_ptr);

        for (size_t i = 0; i < block_size_; ++i)
        {
            int sym = mdl.icdf(RansDecGet(&state, freq_scale_bits_));
            byte_decoded_cb(sym);
            int start = mdl.cdf(sym);
            int freq = mdl.cdf(sym + 1) - start;
            RansDecAdvance(&state, &decode_ptr, start, freq, freq_scale_bits_);
            own_checksum += (uint8_t)sym;
        }
        own_checksum &= 0x7;
        if (checksum != own_checksum)
        {
            printf("checksum mismatch %d, %d\n", checksum, own_checksum);
        }
        return decode_ptr - compressed_data;
    }

private:
    int freq_scale_bits_;
    size_t block_size_;
};