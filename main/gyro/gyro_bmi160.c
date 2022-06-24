// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_bmi160.h"
#include "gyro_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>
#include <driver/timer.h>

#include "bus/bus_i2c.h"

#include "global_context.h"

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
    static int16_t gyro[3] = {0, 0, 0};
    static int16_t accel[3] = {0, 0, 0};
    static bool have_gyro = false, have_accel = false, pipeline_reset = false;

    time += 440;

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
        return false;
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
        gyro_sample_message msg = {.timestamp = time,
                                   .accel_x = accel[0],
                                   .accel_y = accel[1],
                                   .accel_z = accel[2],
                                   .gyro_x = gyro[0],
                                   .gyro_y = gyro[1],
                                   .gyro_z = gyro[2],
                                   .smpl_interval_ns = tmp_data[0],
                                   .flags = (have_accel ? GYRO_SAMPLE_NEW_ACCEL_DATA : 0) |
                                            (pipeline_reset ? GYRO_SAMPLE_PIPELINE_RESET : 0)};
        have_gyro = false;
        have_accel = false;
        pipeline_reset = false;
        if (xQueueSendToBackFromISR(gctx.gyro_raw_queue, &msg, &high_task_awoken) ==
            errQUEUE_FULL) {
            while (1)
                ;
        }
    }
    return high_task_awoken;
}

void gyro_bmi160_task(void* params) {
    gctx.gyro_raw_to_rads = (1.0 / 32.8 * 3.141592 / 180.0);
    gctx.accel_raw_to_g = (16.0 / 32767);
    gctx.gyro_interp_interval = 600;
    gctx.gyro_decimate = 3;
    static uint8_t data[2];
    i2c_register_read(dev_adr, 0x00, data, 1);
    ESP_LOGI(TAG, "WHO_AM_I @ 0x%02X = 0x%02X", dev_adr, data[0]);

    for (int i = 0; i < 10; ++i) {
        i2c_register_write_byte(dev_adr, REG_CMD, 0xb6);  // reset
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    i2c_register_write_byte(dev_adr, REG_CMD, 0xb0);  // fifo reset

    vTaskDelay(100);

    i2c_register_write_byte(dev_adr, REG_ACC_CONF, 0b00101000);
    i2c_register_write_byte(dev_adr, REG_ACC_RANGE, 0b1100);
    i2c_register_write_byte(dev_adr, REG_GYR_CONF, 0b00101100);
    i2c_register_write_byte(dev_adr, REG_GYR_RANGE, 1);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    i2c_register_write_byte(dev_adr, REG_CMD, 0b00010001);  // start accel
    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_register_write_byte(dev_adr, REG_CMD, 0b00010101);  // start gyro
    vTaskDelay(100 / portTICK_PERIOD_MS);

    i2c_register_write_byte(dev_adr, REG_ACC_RANGE, 0b1100);
    i2c_register_write_byte(dev_adr, REG_GYR_RANGE, 1);

    i2c_register_write_byte(dev_adr, REG_FIFO_CONFIG_0, 0);
    i2c_register_write_byte(dev_adr, REG_FIFO_CONFIG_1, 0b11010000);

    // int xx = 0;
    // while (1) {
    //     uint8_t tmp_data[16];
    //     xx++;
    //     if (xx % 500 == 0) {
    //         // ESP_LOGI(TAG, "td0: 0x%02X", tmp_data[0]);
    //         // i2c_register_read(dev_adr, 0x02, tmp_data, 1);
    //         // ESP_LOGI(TAG, "err: 0x%02X", tmp_data[0]);
    //         // i2c_register_read(dev_adr, 0x03, tmp_data, 1);
    //         // ESP_LOGI(TAG, "pmu: 0x%02X", tmp_data[0]);
    //         // i2c_register_read(dev_adr, REG_FIFO_CONFIG_1, tmp_data, 1);
    //         // ESP_LOGI(TAG, "fif_cfg1: 0x%02X", tmp_data[0]);
    //         // i2c_register_read(dev_adr, REG_GYR_CONF, tmp_data, 1);
    //         // ESP_LOGI(TAG, "gyr_conf: 0x%02X", tmp_data[0]);
    //         // i2c_register_read(dev_adr, REG_FIFO_LENGTH_0, tmp_data, 2);
    //         const int fifo_bytes = tmp_data[0] | ((tmp_data[1] & 0x03) << 8);
    //         ESP_LOGI(TAG, "fif_bytes: %d", fifo_bytes);
    //         vTaskDelay(50);
    //     }

    //     i2c_register_read(dev_adr, REG_FIFO_DATA, tmp_data, 1);
    //     uint8_t fh_mode = tmp_data[0] >> 6;
    //     uint8_t fh_parm = (tmp_data[0] >> 2) & 0x0f;
    //     if (tmp_data[0] == 0x80) {  // invalid / empty frame
    //         i2c_register_read(dev_adr, REG_FIFO_DATA, tmp_data, 2);
    //         vTaskDelay(1);
    //         continue;
    //     } else if (fh_mode == 2) {  // regular frame
    //         int bytes_to_read = 1;
    //         if (fh_parm & 1) {  // have acc
    //             bytes_to_read += 6;
    //         }
    //         if (fh_parm & 2) {  // have gyr
    //             bytes_to_read += 6;
    //         }
    //         if (fh_parm & 4) {  // have mag
    //             bytes_to_read += 6;
    //         }
    //         i2c_register_read(dev_adr, REG_FIFO_DATA, tmp_data, bytes_to_read);
    //     } else if (fh_mode == 1) {  // control frame
    //         if (fh_parm == 0) {     // skip
    //             i2c_register_read(dev_adr, REG_FIFO_DATA, tmp_data, 2);
    //             ESP_LOGE(TAG, "skip %d", tmp_data[1]);
    //             i2c_register_write_byte(dev_adr, REG_CMD, 0xb0);  // fifo reset

    //         } else if (fh_parm == 1) {  // sensortime
    //             // TODO
    //             abort();
    //         } else if (fh_parm == 2) {  // fifo input config
    //             // TODO
    //             abort();
    //         } else {
    //             ESP_LOGI(TAG, "unk: 0x%02X", tmp_data[0]);
    //         }
    //     }
    //     // ESP_LOGI(TAG, "fifo: 0x%02X", tmp_data[0]);
    // }

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, .44e-3 * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, gyro_timer_cb, NULL, ESP_INTR_FLAG_IRAM);

    timer_start(TIMER_GROUP_0, TIMER_0);

    vTaskDelete(NULL);
}