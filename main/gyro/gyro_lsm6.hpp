// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once
#include <cstdint>

bool probe_lsm6(uint8_t dev_adr);
void gyro_lsm6_task(void* params);