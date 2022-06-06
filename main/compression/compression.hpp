#pragma once

#include "quat.hpp"

#include <vector>
#include <algorithm>

static inline int8_t iq_i16_i8(double update, double scale)
{
    return std::min(std::max(fabs(update) * scale * ((update < 0) ? -1.0 : 1.0), -127.), 127.);
}

static inline double deiq_i16_i8(int8_t update, double scale)
{
    return (abs(update) / scale) * ((update < 0) ? -1 : 1);
}

struct QuatEncoder
{
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

    std::vector<int8_t> &get_bytes() { return bytes; }

    void reset()
    {
        q_state = {1, 0, 0, 0};
        v_state = {0, 0, 0};
        bytes.clear();
    }

    void clear() { bytes.clear(); }

private:
    double error_limit{0.04};
    double gyro_scale{3000};

    quat::quat q_state{1, 0, 0, 0};
    quat::vec v_state{0, 0, 0};

    std::vector<int8_t> bytes;

    inline void emit_byte(int8_t x) { bytes.push_back(x); }
};

struct QuatDecoder
{
    explicit QuatDecoder() {}

    template <class I>
    bool decode(I data_begin, I data_end,
                const std::function<void(const quat::quat &)> &quat_callback)
    {
        if (wait_header)
        {
            if (data_begin == data_end)
                return false;
            uint16_t gyro_scale_int = *data_begin;
            ++data_begin;
            if (data_begin == data_end)
                return false;
            gyro_scale_int <<= 8;
            gyro_scale_int |= *data_begin;
            ++data_begin;
            gyro_scale = gyro_scale_int * 10;
            wait_header = false;
        }

        double max_error = 0;
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
    bool wait_header{true};
    double gyro_scale = 0;

    quat::quat q_state{1, 0, 0, 0};
    quat::vec v_state{0, 0, 0};
};

template <class I>
bool decode_quats(I data_begin, I data_end,
                  const std::function<void(const quat::quat &)> &quat_callback)
{
    double gyro_scale = 0;
    quat::quat qstate{1, 0, 0, 0};
    quat::vec vstate{0, 0, 0};

    {
        if (data_begin == data_end)
            return false;
        uint16_t gyro_scale_int = *data_begin;
        ++data_begin;
        if (data_begin == data_end)
            return false;
        gyro_scale_int <<= 8;
        gyro_scale_int |= *data_begin;
        ++data_begin;
        gyro_scale = gyro_scale_int * 10;
    }

    double max_error = 0;
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
        vstate += v_dequant;

        qstate = quat::normalize(quat::prod(qstate, quat::from_aa(vstate)));

        if (it != data_end)
        {
            ++it;
        }
        else
        {
            quat_callback(qstate);
            break;
        }

        if (*it == -128)
        {
            ++it;
            continue;
        }
        else
        {
            quat_callback(qstate);
        }
    }

    return true;
}