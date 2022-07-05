// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_lsm6.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>
#include <driver/timer.h>

#include "bus/mini_i2c.h"
}

#include "filters/gyro_ring.hpp"
#include "global_context.hpp"

static const char* TAG = "gyro_lsm6";

#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL3_BIT_BDR_GY_0 4
#define REG_FIFO_CTRL3_BIT_BDR_XL_0 0

#define REG_FIFO_CTRL4 0x0A
#define REG_FIFO_CTRL4_BIT_FIFO_MODE_0 0

#define REG_WHO_AM_I 0x0F

#define REG_CTRL1_XL 0x10
#define REG_CTRL1_XL_BIT_ODR_XL_0 4
#define REG_CTRL1_XL_BIT_FS0_XL 2
#define REG_CTRL1_XL_BIT_LPF2_XL_EN 1

#define REG_CTRL2_G 0x11
#define REG_CTRL2_G_BIT_ODR_G0 4
#define REG_CTRL2_G_BIT_FS0_G 2

#define REG_CTRL3_C 0x12
#define REG_CTRL3_C_BIT_BDU 6
#define REG_CTRL3_C_BIT_IF_INC 2
#define REG_CTRL3_C_BIT_SW_RESET 0

#define REG_CTRL4_C 0x13
#define REG_CTRL4_C_BIT_LPF1_SEL_G 1

#define REG_CTRL6_C 0x15
#define REG_CTRL6_C_BIT_FTYPE_0 0

#define REG_FIFO_STATUS1 0x3a

#define REG_FIFO_STATUS2 0x3b
#define REG_FIFO_STATUS2_BIT_FIFO_OVR_IA 6
#define REG_FIFO_STATUS2_BIT_FIFO_FULL_IA 5
#define REG_FIFO_STATUS2_BIT_FIFO_OVR_LATCHED 3
#define REG_FIFO_STATUS2_BIT_DIFF_FIFO_8 0

#define REG_INTERNAL_FREQ_FINE 0x63

#define REG_FIFO_DATA_OUT_TAG 0x78
#define REG_FIFO_DATA_OUT_X_L 0x79
#define REG_FIFO_DATA_OUT_X_H 0x7A
#define REG_FIFO_DATA_OUT_Y_L 0x7B
#define REG_FIFO_DATA_OUT_Y_H 0x7C
#define REG_FIFO_DATA_OUT_Z_L 0x7D
#define REG_FIFO_DATA_OUT_Z_H 0x7E

#define REG_FIFO_DATA_OUT_TAG_VALUE_TAG_GYRO 1
#define REG_FIFO_DATA_OUT_TAG_VALUE_TAG_ACCEL 2

#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

static SemaphoreHandle_t time_mtx;
static uint64_t cur_gyro_time = 0;
static uint64_t prev_gyro_time = 0;
static bool have_gyro = false, have_accel = false, pipeline_reset = false;

static bool IRAM_ATTR pend_read_fifo(int left);

static void IRAM_ATTR gyro_i2c_cb(void* arg) {
    if (gctx.terminate_for_update) {
        return;
    }

    static uint64_t time = 0;
    if (xSemaphoreTakeFromISR(time_mtx, nullptr)) {
        time = cur_gyro_time;
        xSemaphoreGiveFromISR(time_mtx, nullptr);
    }

    static uint8_t tmp_data[7];
    static int16_t gyro[3];
    static int16_t accel[3];

    static constexpr int64_t kGyroPrescale =
        32768 * (35e-3 * 3.141592 / 180.0) / (1.0 / 32.8 * 3.141592 / 180.0);
    static constexpr int64_t kAccelPrescale = 32768 * 0.488e-3 / (16.0 / 32767);

    mini_i2c_read_reg_get_result(tmp_data, 7);
    uint8_t tag = (tmp_data[0] >> 3);
    if (tag == REG_FIFO_DATA_OUT_TAG_VALUE_TAG_GYRO) {
        gyro[0] = (int16_t)((tmp_data[2] << 8) | tmp_data[1]) * kGyroPrescale / 32768;
        gyro[1] = (int16_t)((tmp_data[4] << 8) | tmp_data[3]) * kGyroPrescale / 32768;
        gyro[2] = (int16_t)((tmp_data[6] << 8) | tmp_data[5]) * kGyroPrescale / 32768;
        have_gyro = true;
    } else if (tag == REG_FIFO_DATA_OUT_TAG_VALUE_TAG_ACCEL) {
        accel[0] = (int16_t)((tmp_data[2] << 8) | tmp_data[1]) * kAccelPrescale / 32768;
        accel[1] = (int16_t)((tmp_data[4] << 8) | tmp_data[3]) * kAccelPrescale / 32768;
        accel[2] = (int16_t)((tmp_data[6] << 8) | tmp_data[5]) * kAccelPrescale / 32768;
        have_accel = true;
    }
    if (have_gyro) {
        gctx.gyro_ring->Push((time - prev_gyro_time) * 1000, gyro[0], gyro[1], gyro[2], accel[0],
                             accel[1], accel[2], have_accel ? kFlagHaveAccel : 0);
        have_gyro = false;
        have_accel = false;
        pipeline_reset = false;
        prev_gyro_time = time;
    }
    int left = (int)arg;
    if (left - 1 > 0) {
        pend_read_fifo(left - 1);
    }
}

static bool IRAM_ATTR pend_read_fifo(int left) {
    return mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_DATA_OUT_TAG, 7, gyro_i2c_cb,
                                      (void*)left);
}

static bool IRAM_ATTR gyro_timer_cb(void* args) {
    if (gctx.terminate_for_update) {
        return false;
    }

    static uint64_t time = 0;
    if (xSemaphoreTakeFromISR(time_mtx, nullptr)) {
        cur_gyro_time = time;
        xSemaphoreGiveFromISR(time_mtx, nullptr);
    }
    time += 220;

    if (gctx.pause_polling) {
        // TODO: fix this
        if (mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CTRL4,
                                    (0 << REG_FIFO_CTRL4_BIT_FIFO_MODE_0)) == ESP_OK) {
            gctx.pause_polling = false;
            return false;
        }
    } else if (gctx.continue_polling) {
        if (mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CTRL4,
                                    (1 << REG_FIFO_CTRL4_BIT_FIFO_MODE_0)) == ESP_OK) {
            gctx.continue_polling = false;
            pipeline_reset = true;
        }
    }

    static uint8_t tmp_data[7];

    if (mini_i2c_read_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_STATUS1, tmp_data, 2) != ESP_OK) {
        return false;
    }
    int fifo_bytes = tmp_data[0] | ((tmp_data[1] & 0x03) << 8);

    if (fifo_bytes > 0) {
        pend_read_fifo(fifo_bytes);
    }

    if (fifo_bytes > 256) {
        while (1)
            ;
    }
    return false;
}

bool probe_lsm6(uint8_t dev_adr) {
    static uint8_t data[1];
    if (mini_i2c_read_reg_sync(dev_adr, REG_WHO_AM_I, data, 1) != ESP_OK) {
        ESP_LOGW("gyro-prober", "read failed");
        return false;
    }
    ESP_LOGI("gyro-prober", "ok %02X", data[0]);
    return data[0] == 0x6b;
}

void gyro_lsm6_task(void* params) {
    time_mtx = xSemaphoreCreateMutex();
    mini_i2c_set_timing(500000);

    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CTRL3_C, (3 << REG_CTRL3_C_BIT_SW_RESET));

    vTaskDelay(5);

    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CTRL3_C,
                            (3 << REG_CTRL3_C_BIT_BDU) | (3 << REG_CTRL3_C_BIT_IF_INC));

    // clang-format off
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CTRL6_C, (3 << REG_CTRL6_C_BIT_FTYPE_0));
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CTRL4_C, (1 << REG_CTRL4_C_BIT_LPF1_SEL_G));
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CTRL2_G, (9 << REG_CTRL2_G_BIT_ODR_G0) | (2 << REG_CTRL2_G_BIT_FS0_G)); // 9 is 3.33 khz
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CTRL1_XL, (5 << REG_CTRL1_XL_BIT_ODR_XL_0) | (1 << REG_CTRL1_XL_BIT_FS0_XL)); // 5 is 208 hz
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CTRL3, (9 << REG_FIFO_CTRL3_BIT_BDR_GY_0) | (5 << REG_FIFO_CTRL3_BIT_BDR_XL_0));
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CTRL4, (1 << REG_FIFO_CTRL4_BIT_FIFO_MODE_0));
    // clang-format on

    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, .22e-3 * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, gyro_timer_cb, NULL, ESP_INTR_FLAG_IRAM);

    timer_start(TIMER_GROUP_0, TIMER_0);

    vTaskDelete(NULL);
}