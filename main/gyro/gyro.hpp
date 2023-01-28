// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

#include "gyro_mpu6050.hpp"
#include "gyro_bmi160.hpp"
#include "gyro_icm42688.hpp"


struct GyroHal {
    bool ready{};
    int terminate{};
    uint8_t i2c_adr{};
    char const* gyro_type{};

    float gyro_sr{};
    int accel_div{};

    void (*gyro_cb)(int16_t* gyr, uint32_t dt, void* ctx);
    void (*accel_cb)(int16_t* accel, void* ctx);
    void* cb_ctx;
};

bool gyro_hal_init(GyroHal* hal, int sda, int scl);