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
class SettingsManager;

struct FilterSettings {
    bool disable_accel{};
    int pt_order{};
    int pt_cutoff{};

    int accel_pt_order{};
    double accel_pt_cutoff{};

    int dyn_count{};
    int dyn_freq_min{};
    int dyn_freq_max{};
    int dyn_q{};
    int dyn_lr{};
    double dyn_lr_smooth{}; 
};

typedef struct {
    GyroRing* gyro_ring;
    SettingsManager* settings_manager;

    uint8_t gyro_i2c_adr;
    double gyro_sr;
    double accel_sr;

    FilterSettings filter_settings;

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
        uint32_t avg_sample_interval_ns;
        uint64_t last_block_time_us;
        bool storage_failure;

        SemaphoreHandle_t accel_raw_mtx;
        double accel_raw[3];

        float incline_error;
    } logger_control;

} GlobalContext;

extern GlobalContext gctx;