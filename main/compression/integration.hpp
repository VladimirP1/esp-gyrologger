#pragma once

extern "C" {
#include "gyro/gyro_types.h"
#include "global_context.h"
}
#include "lib/fixquat.hpp"

#include <vector>

struct BasicIntegrator {
    explicit BasicIntegrator(int block_size, int decimate)
        : block_size(block_size), decimate(decimate) {}

    bool update(const gyro_sample_message& sample) {
        if (first_run) prev_ts = sample.timestamp;
        double scale = gctx.gyro_raw_to_rads * (sample.timestamp - prev_ts) / 1e6;
        quat::vec gyro{quat::base_type{sample.gyro_x * scale},
                       quat::base_type{sample.gyro_y * scale},
                       quat::base_type{sample.gyro_z * scale}};
        current_quat = (quat::quat(gyro) * current_quat).normalized();
        prev_ts = sample.timestamp;
        first_run = false;
        if (sample_counter++ % decimate == 0) {
            quats.push_back(current_quat);
        }
        return quats.size() == block_size;
    }

    void clear() { quats.clear(); }

    quat::quat* data() { return quats.data(); }

    std::vector<quat::quat> quats;

   private:
    quat::quat current_quat;

    unsigned long long prev_ts{};
    bool first_run{true};
    int sample_counter{};

    int block_size;
    int decimate;
};