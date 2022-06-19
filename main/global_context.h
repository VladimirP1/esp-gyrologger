#pragma once

#include "gyro/gyro_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define kBlockSize 256

typedef struct {
    QueueHandle_t gyro_raw_queue;
    QueueHandle_t gyro_interp_queue;
    double gyro_raw_to_rads;
    struct {
        SemaphoreHandle_t mutex;

        bool active;
        char* file_name;

        bool broken;

        bool calibration_pending;
        
        // logger stats
        bool busy;
        int total_samples_written;
        int total_bytes_written;
        uint64_t log_start_ts_ms;
        int avg_logging_rate_bytes_min;
    } logger_control;

} GlobalContext;

extern GlobalContext gctx;