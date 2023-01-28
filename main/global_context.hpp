// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
}

#define kBlockSize 256
typedef struct {
    uint8_t gyro_i2c_adr;
    QueueHandle_t aux_i2c_queue;
} GlobalContext;

extern GlobalContext gctx;