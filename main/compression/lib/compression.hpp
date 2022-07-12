// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

#include "fixquat.hpp"
#include "rans_byte.h"
#include "laplace_model.inc"

// #include <iostream>
#include <vector>
#include <tuple>

class Coder {
   private:
    struct single_update {
        int8_t x, y, z;
        bool is_saturated() const { return abs(x) == 127 || abs(y) == 127 || abs(z) == 127; };
    };

    struct {
        quat::quat q;
        quat::vec v;
    } state;

    struct {
        int8_t scale{22};
    } qp;

    int block_size;
    quat::base_type target_quality_deg{-1};
    int target_block_size{};
    bool enable_pressure{};
    int pressure_cnt{};

    std::vector<uint8_t> compressed_block;

   private:
    static inline single_update quant_update(quat::vec update, int8_t scale) {
        int8_t ux = update.x.raw_value() >> scale;
        int8_t uy = update.y.raw_value() >> scale;
        int8_t uz = update.z.raw_value() >> scale;

        using T = decltype(update.x);
        if (ux != (update.x.raw_value() >> scale)) ux = update.x < T{0} ? -127 : 127;
        if (uy != (update.y.raw_value() >> scale)) uy = update.y < T{0} ? -127 : 127;
        if (uz != (update.z.raw_value() >> scale)) uz = update.z < T{0} ? -127 : 127;
        return single_update{ux, uy, uz};
    }

    static inline quat::vec dequant_update(single_update update, int8_t scale) {
        using R = quat::base_type;
        return {R::from_raw_value(((int)update.x) << scale),
                R::from_raw_value(((int)update.y) << scale),
                R::from_raw_value(((int)update.z) << scale)};
    }

    bool update_qp(int bytes_put, quat::base_type max_error, int tr) {
        if (target_block_size < 0) {
            if (qp.scale != -target_block_size) {
                qp.scale = -target_block_size;
                return true;
            } else {
                return false;
            }
        }
        if (enable_pressure && (tr == 0) && (++pressure_cnt % 10 == 0)) {
            qp.scale += 1;
        } else {
            if (target_quality_deg >= quat::base_type{0} && !target_block_size) {
                if (max_error > target_quality_deg) {
                    qp.scale -= 1;
                } else if (!enable_pressure &&
                           max_error < target_quality_deg / quat::base_type{1.5}) {
                    qp.scale += 1;
                } else {
                    return false;
                }
            } else if (target_quality_deg >= quat::base_type{0}) {
                if (bytes_put > target_block_size) {
                    qp.scale += 1;
                } else if (max_error > target_quality_deg) {
                    qp.scale -= 1;
                } else if (max_error < target_quality_deg / quat::base_type{1.5}) {
                    qp.scale += 1;
                } else {
                    return false;
                }
            } else {
                if (bytes_put > target_block_size) {
                    qp.scale += 1;
                } else if (bytes_put < target_block_size / 1.5) {
                    qp.scale -= 1;
                } else {
                    return false;
                }
            }
        }
        qp.scale = std::max(std::min((int)qp.scale, 20), 8);
        return true;
    }

   public:
    struct BitrateModeConstantQP {};
    struct BitrateModeConstantQuality {};
    struct BitrateModeConstantQualityWithPressure {};
    struct BitrateModeConstantBitrate {};
    struct BitrateModeConstantQualityLimited {};

    explicit Coder(int block_size, BitrateModeConstantQP m, int qp)
        : block_size(block_size), target_block_size(-qp) {}

    explicit Coder(int block_size, BitrateModeConstantBitrate m, int target_block_size)
        : block_size(block_size), target_block_size(target_block_size) {}

    explicit Coder(int block_size, BitrateModeConstantQuality m, double target_quality_deg)
        : block_size(block_size), target_quality_deg(target_quality_deg) {}

    explicit Coder(int block_size, BitrateModeConstantQualityWithPressure m,
                   double target_quality_deg)
        : block_size(block_size), target_quality_deg(target_quality_deg), enable_pressure(true) {}

    explicit Coder(int block_size, BitrateModeConstantQualityLimited m, double target_quality_deg,
                   int max_block_size)
        : block_size(block_size),
          target_quality_deg(target_quality_deg),
          target_block_size(max_block_size) {}

    explicit Coder(int block_size) : block_size(block_size) {}

    std::pair<std::vector<uint8_t>&, quat::base_type> encode_block(quat::quat* quats) {
        struct UpdateEncoder {
            std::vector<int8_t> buf;

            explicit UpdateEncoder(decltype(qp.scale) scale) : scale(scale) {}

            quat::vec encode_update(quat::vec v) {
                quat::vec sum;
                for (bool correction_needed = true; correction_needed;) {
                    single_update update_quanted = quant_update(v, scale);
                    quat::vec update_dequanted = dequant_update(update_quanted, scale);

                    v -= update_dequanted;
                    sum += update_dequanted;

                    correction_needed = update_quanted.is_saturated();

                    emit(update_quanted);
                }
                return sum;
            }

           private:
            int8_t scale;

            void emit(single_update upd) {
                buf.push_back(upd.x);
                buf.push_back(upd.y);
                buf.push_back(upd.z);
                // std::cout << "single_update {" << (int)upd.x << ", " << (int)upd.y << ", "
                //   << (int)upd.z << "}" << std::endl;
            }
        };

        auto old_state = state;
        size_t bytes_put{};
        quat::base_type max_angle_error_rad{};
        compressed_block.resize(2000);
        for (int tr = 0; tr < 5; ++tr) {
            state = old_state;

            UpdateEncoder upd_enc(qp.scale);

            max_angle_error_rad = {};
            for (int i = 0; i < block_size; ++i) {
                auto& q = quats[i];
                quat::quat q_update = state.q.conj() * q;
                quat::vec v_update = q_update.axis_angle() - state.v;

                quat::vec decoded_update = upd_enc.encode_update(v_update);

                state.v += decoded_update;
                state.q = (state.q * quat::quat{state.v}).normalized();

                max_angle_error_rad =
                    std::max((state.q.conj() * q).axis_angle().norm(), max_angle_error_rad);
            }

            double std = 0;
            {
                uint64_t sum = 0;
                for (auto& sym : upd_enc.buf) {
                    sum += sym * sym;
                }
                std = double(sum) / upd_enc.buf.size();
            }

            auto it = std::lower_bound(kVarianceTable.begin(), kVarianceTable.end(), std);
            if (it == kVarianceTable.end()) {
                it = kVarianceTable.begin() + kVarianceTable.size() - 1;
            }
            int i_var = std::distance(kVarianceTable.begin(), it);

            // std::cout << upd_enc.buf.size() << " " << i_var << std::endl;

            LaplaceModel mdl(i_var);
            RansState state;

            uint8_t* compressed_ptr = compressed_block.data() + compressed_block.size();
            RansEncInit(&state);

            bool failure = false;
            uint8_t checksum = 0;
            auto data_begin = upd_enc.buf.begin();
            auto data_end = upd_enc.buf.end();
            while (data_begin != data_end) {
                --data_end;
                int start = mdl.cdf(*data_end);
                int freq = mdl.cdf(*data_end + 1) - start;
                RansEncPut(&state, &compressed_ptr, start, freq, 15);
                checksum += static_cast<uint8_t>(*data_end);
                if (compressed_ptr - compressed_block.data() < 16) {
                    printf("scale = %d, i_var = %d\n", qp.scale, i_var);
                    // abort();
                    qp.scale = std::min(qp.scale + 2, 22);
                    --tr;
                    failure = true;
                    break;
                }
            }
            if (failure) continue;
            RansEncFlush(&state, &compressed_ptr);
            *(--compressed_ptr) = i_var | (checksum << 5);
            *(--compressed_ptr) = qp.scale;

            bytes_put = compressed_block.data() + compressed_block.size() - compressed_ptr;

            if (!update_qp(bytes_put, max_angle_error_rad, tr)) {
                break;
            }
        }
        // std::cout << "Put " << bytes_put
        //           << " bytes; max error = " << double(max_angle_error_rad) * 180.0 / M_PI
        //           << std::endl;

        compressed_block.erase(compressed_block.begin(), compressed_block.end() - bytes_put);

        return {compressed_block, max_angle_error_rad};
    }

    std::tuple<size_t, std::vector<quat::quat>, int> decode_block(uint8_t* compressed_data) {
        int scale = compressed_data[0];
        int i_var = compressed_data[1] & 0x1f;
        uint8_t checksum = compressed_data[1] >> 5;
        uint8_t own_checksum = 0;

        LaplaceModel mdl(i_var);
        RansState rans_state;
        uint8_t* decode_ptr = compressed_data + 2;
        RansDecInit(&rans_state, &decode_ptr);

        std::vector<quat::quat> quats;
        quats.reserve(block_size);
        while (quats.size() < block_size) {
            single_update upd{};
            int8_t* upd_ptr = &upd.x;
            uint8_t upd_idx = 0;
            for (size_t i = 0; i < 3; ++i) {
                int sym = mdl.icdf(RansDecGet(&rans_state, 15));
                upd_ptr[upd_idx++] = sym;
                int start = mdl.cdf(sym);
                int freq = mdl.cdf(sym + 1) - start;
                RansDecAdvance(&rans_state, &decode_ptr, start, freq, 15);
                own_checksum += (uint8_t)sym;
            }

            state.v += dequant_update(upd, scale);

            if (!upd.is_saturated()) {
                state.q = (state.q * quat::quat{state.v}).normalized();
                quats.push_back(state.q);
            }
            // std::cout << "pos = " << decode_ptr - compressed_data << std::endl;
        }
        own_checksum &= 0x7;
        if (checksum != own_checksum) {
            // printf("checksum mismatch %d, %d\n", checksum, own_checksum);
            abort();
        }
        return {decode_ptr - compressed_data, quats, scale};
    }

    void reset() {
        state.q = {};
        state.v = {};
    }
};