// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once
#include <cstdint>

bool probe_bmi160(uint8_t dev_adr);
void gyro_bmi160_task(void* params);