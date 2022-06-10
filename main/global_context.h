#pragma once

#include "gyro/gyro_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

typedef struct {
    QueueHandle_t gyro_raw_queue;
    struct {
        SemaphoreHandle_t mutex;

        bool active;
        char* file_name;

        bool broken;

        bool calibration_pending;
        
        //---
        bool busy;
    } logger_control;

} GlobalContext;

extern GlobalContext gctx;