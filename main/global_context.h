#pragma once

#include "gyro/gyro_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

typedef struct {
    QueueHandle_t gyro_raw_queue;
    
} GlobalContext;

extern GlobalContext ctx;