#pragma once

extern "C"
{
#include "gyro/gyro_types.h"
}
#include "quat.hpp"

struct BasicIntegrator
{
    explicit BasicIntegrator() {}

    quat::quat update(const gyro_sample_message &sample)
    {
        if (first_run)
            prev_ts = sample.timestamp;
        quat::vec gyro{sample.gyro_x * kMessageGyroScale, sample.gyro_y * kMessageGyroScale, sample.gyro_z * kMessageGyroScale};
        current_quat = quat::normalize(
            quat::prod(quat::from_aa(quat::map(gyro, [&](double x)
                                               { return x * (sample.timestamp - prev_ts) / 1e6; })),
                       current_quat));
        prev_ts = sample.timestamp;
        first_run = false;
        return current_quat;
    }

private:
    unsigned long long prev_ts{};
    quat::quat current_quat{1, 0, 0, 0};

    bool first_run{true};
};