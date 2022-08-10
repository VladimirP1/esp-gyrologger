// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_bmi160.hpp"

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

#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

static void IRAM_ATTR update_alarm(uint64_t dt) {
    timer_group_set_counter_enable_in_isr(TIMER_GROUP_0, TIMER_0, TIMER_PAUSE);
    uint64_t alarm = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0) + dt;
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, alarm);
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_set_counter_enable_in_isr(TIMER_GROUP_0, TIMER_0, TIMER_START);
}

static uint64_t prev_gyro_time = 0;

static int bytes_to_read = 1;
static void IRAM_ATTR gyro_i2c_cb(void* args) {
    if (gctx.terminate_for_update) {
        return;
    }

    uint64_t time = esp_timer_get_time();

    static uint8_t tmp_data[13];
    static uint8_t fh_mode = 0, fh_parm = 0;
    static int16_t gyro[3] = {0, 0, 0};
    static int16_t accel[3] = {0, 0, 0};
    static bool have_gyro = false, have_accel = false, pipeline_reset = false;

    if (mini_i2c_read_reg_get_result(tmp_data, bytes_to_read) == ESP_OK) {
        if (bytes_to_read == 1) {
            fh_mode = tmp_data[0] >> 6;
            fh_parm = (tmp_data[0] >> 2) & 0x0f;
            if (tmp_data[0] == 0x80) {  // invalid / empty frame
            } else if (fh_mode == 2) {  // regular frame
                bytes_to_read = 1;
                if (fh_parm & 4) {  // have mag
                    bytes_to_read += 6;
                }
                if (fh_parm & 2) {  // have gyr
                    bytes_to_read += 6;
                }
                if (fh_parm & 1) {  // have acc
                    bytes_to_read += 6;
                }
            } else if (fh_mode == 1) {  // control frame
                if (fh_parm == 0) {     // skip
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
        } else {
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
            bytes_to_read = 1;
        }
    } else {
        mini_i2c_hw_fsm_reset();
    }
    if (have_gyro) {
        gctx.gyro_ring->Push((time - prev_gyro_time) * 1000, gyro[0], gyro[1], gyro[2], accel[0],
                             accel[1], accel[2], have_accel ? kFlagHaveAccel : 0);
        have_gyro = false;
        have_accel = false;
        pipeline_reset = false;
        prev_gyro_time = time;
    }

    mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_DATA, bytes_to_read, gyro_i2c_cb,
                               nullptr);
}

bool probe_bmi160(uint8_t dev_adr) {
    uint8_t data[1];
    if (mini_i2c_read_reg_sync(dev_adr, 0x00, data, 1) != ESP_OK) {
        return false;
    }
    return data[0] == 0xD1;
}

void gyro_bmi160_task(void* params) {
    gctx.gyro_sr = 3300.0;
    gctx.accel_sr = 100.0;

    for (int i = 0; i < 10; ++i) {
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0xb6);  // reset
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0xb0);  // fifo reset

    vTaskDelay(100 / portTICK_PERIOD_MS);

    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0b00010001);  // start accel
    vTaskDelay(10 / portTICK_PERIOD_MS);
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0b00010101);  // start gyro
    vTaskDelay(100 / portTICK_PERIOD_MS);

    mini_i2c_set_timing(710000);

    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_ACC_RANGE, 0b1100);
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYR_RANGE, 1);
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_ACC_CONF, 0b00101000); // 100hz sample rate, OSR1
    // mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYR_CONF, 0b00001101); // 3.2k sample rate, OSR4 (254hz LPF)
    // mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYR_CONF, 0b00011101); // 3.2k sample rate, OSR2 (524hz LPF)
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYR_CONF, 0b00101101); // 3.2k sample rate, OSR1 (890hz LPF)


    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CONFIG_0, 0);
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CONFIG_1, 0b11010000);

    mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_DATA, bytes_to_read, gyro_i2c_cb,
                               nullptr);

    vTaskDelete(NULL);
}