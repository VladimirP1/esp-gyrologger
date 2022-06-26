// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
}

#define kBlockSize 256

class GyroRing;
typedef struct {
    GyroRing* gyro_ring;

    volatile bool pause_polling;
    volatile bool continue_polling;
    volatile bool terminate_for_update;
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