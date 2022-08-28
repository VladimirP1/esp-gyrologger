#pragma once
#include <cstdint>

bool probe_icm42688(uint8_t dev_adr);

void gyro_icm42688_task(void *params_pvoid);