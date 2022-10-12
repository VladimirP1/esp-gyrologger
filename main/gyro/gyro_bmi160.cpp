// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_bmi160.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>
#include <driver/timer.h>
#include <esp_timer.h>

#include "bus/mini_i2c.h"
}

#include "bmi270_cfg.h"

#include "filters/gyro_ring.hpp"
#include "global_context.hpp"
#include "bus/aux_i2c.hpp"

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

// bmi270 specific
#define REG_INIT_CTRL 0x59

#define REG_PWR_CONF 0x7c

#define REG_PWR_CTRL 0x7d

#define REG_FIFO_DATA_BMI270 0x26

#define REG_FIFO_CONFIG_0_BMI270 0x48

#define REG_FIFO_CONFIG_1_BMI270 0x49

static bool is_bmi270 = false;
static uint8_t factor_zx{};

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
    static uint64_t prev_gyro_time = 0;
    static bool have_gyro = false, have_accel = false;

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
                    bytes_to_read = 5;
                }
            }
        } else {
            int i = 1;
            if (fh_parm & 4) {  // have mag
                i += 6;
            }
            if (fh_parm & 2) {  // have gyr
                gyro[2] = (int16_t)((tmp_data[i + 5] << 8) | tmp_data[i + 4]);
                gyro[1] = (int16_t)((tmp_data[i + 3] << 8) | tmp_data[i + 2]);
                gyro[0] = (int16_t)((tmp_data[i + 1] << 8) | tmp_data[i + 0]) -
                          factor_zx * gyro[2] / (1 << 9);
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
        prev_gyro_time = time;
    }

    proc_aux_i2c();
    mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, is_bmi270 ? REG_FIFO_DATA_BMI270 : REG_FIFO_DATA,
                               bytes_to_read, gyro_i2c_cb, nullptr);
}

bool probe_bmi160(uint8_t dev_adr) {
    uint8_t data[1];
    if (mini_i2c_read_reg_sync(dev_adr, 0x00, data, 1) != ESP_OK) {
        return false;
    }
    return data[0] == 0xD1 || data[0] == 0x24;
}

static bool check_is_bmi270() {
    uint8_t data[1];
    if (mini_i2c_read_reg_sync(gctx.gyro_i2c_adr, 0x00, data, 1) != ESP_OK) {
        return false;
    }
    return data[0] == 0x24;
}

void gyro_bmi160_task(void* params) {
    gctx.gyro_sr = 3300.0;
    gctx.accel_sr = 100.0;

    for (int i = 0; i < 10; ++i) {
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0xb6);  // reset
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    is_bmi270 = check_is_bmi270();
    if (is_bmi270) {
        ESP_LOGI(TAG, "BMI270 detected");
        // disable power save
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_PWR_CONF, 0x00);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        // prepare to load config file
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_INIT_CTRL, 0x00);
        // load config file
        for (int i = 0; i < sizeof(bmi270_config_file); i += 2) {
            mini_i2c_write_reg2_sync(gctx.gyro_i2c_adr, 0x5b, (uint8_t)((i / 2) & 0x0F),
                                     (uint8_t)((i / 2) >> 4));
            mini_i2c_write_reg2_sync(gctx.gyro_i2c_adr, 0x5e, bmi270_config_file[i],
                                     bmi270_config_file[i + 1]);
        }
        // complete loading config file
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_INIT_CTRL, 0x01);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        mini_i2c_set_timing(710000);
        mini_i2c_double_stop_timing();
        mini_i2c_double_stop_timing();
        mini_i2c_double_stop_timing();

        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CONFIG_0_BMI270, 0);
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CONFIG_1_BMI270, 0b11010000);

        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_ACC_CONF,
                                0b10101000);  // 100hz sample rate, OSR1
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_ACC_RANGE, 3);

        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYR_CONF,
                                0b11101101);  // 3.2k sample rate, OSR1 (890hz LPF)
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYR_RANGE, 0b00001001);

        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_PWR_CONF, 0x02);
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_PWR_CTRL, 0x06);  // enable gyro + acc
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0xb0);       // fifo reset

        mini_i2c_read_reg_sync(gctx.gyro_i2c_adr, 0x3c, &factor_zx, 1);

        mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_DATA_BMI270, bytes_to_read,
                                   gyro_i2c_cb, nullptr);
    } else {
        ESP_LOGI(TAG, "BMI160 detected");
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0xb0);  // fifo reset

        vTaskDelay(100 / portTICK_PERIOD_MS);

        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0b00010001);  // start accel
        vTaskDelay(10 / portTICK_PERIOD_MS);
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CMD, 0b00010101);  // start gyro
        vTaskDelay(100 / portTICK_PERIOD_MS);

        mini_i2c_set_timing(710000);

        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_ACC_CONF,
                                0b00101000);  // 100hz sample rate, OSR1
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_ACC_RANGE, 0b1100);
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYR_CONF,
                                0b00101101);  // 3.2k sample rate, OSR1 (890hz LPF)
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYR_RANGE, 1);

        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CONFIG_0, 0);
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CONFIG_1, 0b11010000);

        mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_DATA, bytes_to_read, gyro_i2c_cb,
                                   nullptr);
    }

    vTaskDelete(NULL);
}