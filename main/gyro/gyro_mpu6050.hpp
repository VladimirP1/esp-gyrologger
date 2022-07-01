// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once
#include <cstdint>

bool probe_mpu6050(uint8_t dev_adr);
void gyro_mpu6050_task(void* params);