// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

extern "C" {
#include "gyro/gyro_types.h"
#include "global_context.h"
}
#include "lib/fixquat.hpp"

#include <vector>

struct BasicIntegrator {
    explicit BasicIntegrator(int block_size)
        : block_size(block_size) {}

    bool update(const gyro_sample_message& sample) {
        if (first_run) prev_ts = sample.timestamp;
        double scale = gctx.gyro_raw_to_rads * (sample.timestamp - prev_ts) / 1e6;
        quat::vec gyro{quat::base_type{sample.gyro_x * scale},
                       quat::base_type{sample.gyro_y * scale},
                       quat::base_type{sample.gyro_z * scale}};
        current_quat = (current_quat * quat::quat(gyro));
        prev_ts = sample.timestamp;
        first_run = false;
        current_quat = accel_correction * current_quat;
        if (sample_counter % 10 == 0) {
            current_quat = current_quat.normalized();
        }
        if (sample_counter++ % gctx.gyro_decimate == 0) {
            quats.push_back(current_quat);
        }

        if (sample.flags & GYRO_SAMPLE_NEW_ACCEL_DATA) {
            int threshold = 1.25 / gctx.accel_raw_to_g;
            if (abs(sample.accel_x) < threshold && abs(sample.accel_y) < threshold &&
                abs(sample.accel_z) < threshold) {
                double norm =
                    sqrt(sample.accel_x * sample.accel_x + sample.accel_y * sample.accel_y +
                         sample.accel_z * sample.accel_z);
                quat::vec accel{quat::base_type{sample.accel_x / norm},
                                quat::base_type{sample.accel_y / norm},
                                quat::base_type{sample.accel_z / norm}};
                accel = accel.normalized();
                quat::vec gp = current_quat.rotate_point(accel);
                auto dq0 = (gp.z + quat::base_type{1.0}) * quat::base_type{0.5};
                accel_correction = quat::quat(quat::base_type{dq0}, -gp.y / quat::base_type{2.0},
                                              gp.x / quat::base_type{2.0}, quat::base_type{0.0})
                                       .normalized();

                static uint8_t cnt;
                if (cnt++ == 0)
                    printf("corr: %f\n",
                           ((double)(accel_correction.axis_angle() / quat::base_type{4}).norm()) *
                               4 * 180.0 / M_PI);

                accel_correction =
                    quat::quat{(accel_correction.axis_angle() * quat::base_type{-0.0001})};
            } else {
                accel_correction = {};
            }
        }
        return quats.size() == block_size;
    }

    void clear() { quats.clear(); }

    quat::quat* data() { return quats.data(); }

    std::vector<quat::quat> quats;

   private:
    quat::quat current_quat{};
    quat::quat accel_correction{};

    unsigned long long prev_ts{};
    bool first_run{true};
    int sample_counter{};

    int block_size;
};