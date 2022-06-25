// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_bmi160.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>
#include <driver/timer.h>

#include "bus/bus_i2c.h"
}

#include "filters/gyro_ring.hpp"
#include "global_context.hpp"

static const char* TAG = "gyro_bmi160";

#define REG_FIFO_LENGTH_0 0x22

#define REG_FIFO_LENGTH_1 0x23

#define REG_FIFO_DATA 0x24

#define REG_ACC_CONF 0x40

#define REG_ACC_RANGE 0x41

#define REG_GYR_CONF 0x42

#define REG_GYR_RANGE 0x43

#define REG_FIFO_CONFIG_0 0x46

#define REG_FIFO_CONFIG_1 0x47

#define REG_CMD 0x7e

static uint8_t dev_adr = 0x69;

#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

static bool IRAM_ATTR gyro_timer_cb(void* args) {
    static uint8_t tmp_data[1024];
    static uint64_t time = 0;
    static uint64_t prev_gyro_time = 0;
    static int16_t gyro[3] = {0, 0, 0};
    static int16_t accel[3] = {0, 0, 0};
    static bool have_gyro = false, have_accel = false, pipeline_reset = false;

    if (gctx.pause_polling) {
        i2c_register_write_byte(dev_adr, REG_FIFO_CONFIG_1, 0);
        gctx.pause_polling = false;
        return false;
    } else if (gctx.continue_polling) {
        i2c_register_write_byte(dev_adr, REG_FIFO_CONFIG_1, 0b11010000);
        gctx.continue_polling = false;
        pipeline_reset = true;
    }

    i2c_register_read(dev_adr, REG_FIFO_DATA, tmp_data, 1);
    uint8_t fh_mode = tmp_data[0] >> 6;
    uint8_t fh_parm = (tmp_data[0] >> 2) & 0x0f;
    if (tmp_data[0] == 0x80) {  // invalid / empty frame
    } else if (fh_mode == 2) {  // regular frame
        int bytes_to_read = 1;
        if (fh_parm & 4) {  // have mag
            bytes_to_read += 6;
        }
        if (fh_parm & 2) {  // have gyr
            bytes_to_read += 6;
        }
        if (fh_parm & 1) {  // have acc
            bytes_to_read += 6;
        }
        i2c_register_read(dev_adr, REG_FIFO_DATA, tmp_data, bytes_to_read);

        int i = 1;
        if (fh_parm & 4) {  // have mag
            i += 6;
        }
        if (fh_parm & 2) {  // have gyr
            gyro[0] = (int16_t)((tmp_data[i + 1] << 8) | tmp_data[i + 0]);
            gyro[1] = (int16_t)((tmp_data[i + 3] << 8) | tmp_data[i + 2]);
            gyro[2] = (int16_t)((tmp_data[i + 5] << 8) | tmp_data[i + 4]);
            i += 6;
            have_gyro = true;
        }
        if (fh_parm & 1) {  // have acc
            accel[0] = (int16_t)((tmp_data[i + 1] << 8) | tmp_data[i + 0]);
            accel[1] = (int16_t)((tmp_data[i + 3] << 8) | tmp_data[i + 2]);
            accel[2] = (int16_t)((tmp_data[i + 5] << 8) | tmp_data[i + 4]);
            i += 6;
            have_accel = true;
        }

    } else if (fh_mode == 1) {  // control frame
        if (fh_parm == 0) {     // skip
            i2c_register_read(dev_adr, REG_FIFO_DATA, tmp_data, 2);
            while (1)
                ;
        } else if (fh_parm == 1) {  // sensortime
                                    // TODO
            while (1)
                ;
        } else if (fh_parm == 2) {  // fifo input config
                                    // TODO
            while (1)
                ;
        }
    }

    BaseType_t high_task_awoken = pdFALSE;
    if (have_gyro) {
        gctx.gyro_ring.Push((time - prev_gyro_time) * 1000, gyro[0], gyro[1], gyro[2], accel[0],
                            accel[1], accel[2], have_accel ? kFlagHaveAccel : 0);
        have_gyro = false;
        have_accel = false;
        pipeline_reset = false;
        prev_gyro_time = time;
    }
    time += 440;
    return high_task_awoken;
}

extern "C" {
static bool IRAM_ATTR gyro_timer_cb_c(void* args) { return gyro_timer_cb(args); }
}

void gyro_bmi160_task(void* params) {
    static uint8_t data[2];
    i2c_register_read(dev_adr, 0x00, data, 1);
    ESP_LOGI(TAG, "WHO_AM_I @ 0x%02X = 0x%02X", dev_adr, data[0]);

    for (int i = 0; i < 10; ++i) {
        i2c_register_write_byte(dev_adr, REG_CMD, 0xb6);  // reset
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    i2c_register_write_byte(dev_adr, REG_CMD, 0xb0);  // fifo reset

    vTaskDelay(100 / portTICK_PERIOD_MS);

    i2c_register_write_byte(dev_adr, REG_CMD, 0b00010001);  // start accel
    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_register_write_byte(dev_adr, REG_CMD, 0b00010101);  // start gyro
    vTaskDelay(100 / portTICK_PERIOD_MS);

    i2c_register_write_byte(dev_adr, REG_ACC_RANGE, 0b1100);
    i2c_register_write_byte(dev_adr, REG_GYR_RANGE, 1);
    i2c_register_write_byte(dev_adr, REG_ACC_CONF, 0b00101000);
    i2c_register_write_byte(dev_adr, REG_GYR_CONF, 0b00101100);

    i2c_register_write_byte(dev_adr, REG_FIFO_CONFIG_0, 0);
    i2c_register_write_byte(dev_adr, REG_FIFO_CONFIG_1, 0b11010000);

    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, .44e-3 * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, gyro_timer_cb_c, NULL, ESP_INTR_FLAG_IRAM);

    timer_start(TIMER_GROUP_0, TIMER_0);

    vTaskDelete(NULL);
}