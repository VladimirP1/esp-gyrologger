#pragma once

extern "C" {
#include "gyro/gyro_types.h"
}
#include "lib/fixquat.hpp"

#include <vector>

struct BasicIntegrator {
    explicit BasicIntegrator() {}

    quat::quat update(const gyro_sample_message &sample) {
        if (first_run) prev_ts = sample.timestamp;
        double scale = kMessageGyroScale * (sample.timestamp - prev_ts) / 1e6;
        quat::vec gyro{quat::base_type{sample.gyro_x * scale},
                       quat::base_type{sample.gyro_y * scale},
                       quat::base_type{sample.gyro_z * scale}};
        quat::quat current_quat = quats.back();
        current_quat = (quat::quat(gyro) * current_quat).normalized();
        prev_ts = sample.timestamp;
        first_run = false;
        quats.push_back(current_quat);
        return current_quat;
    }

    void trim(int block_size) { quats.erase(quats.begin(), quats.begin() + block_size); }

    std::vector<quat::quat> quats{1};

   private:
    unsigned long long prev_ts{};
    bool first_run{true};
};