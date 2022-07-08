// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_lsm6.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>

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

#define REG_CTRL9_XL 0x18
#define REG_CTRL9_XL_BIT_I3C_DISABLE 1

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

static bool IRAM_ATTR pend_read_fifo(int left);

static void IRAM_ATTR gyro_i2c_cb(void* arg) {
    if (gctx.terminate_for_update) {
        return;
    }

    static bool have_gyro = false, have_accel = false, pipeline_reset = false;
    uint64_t time = esp_timer_get_time();
    static uint64_t prev_gyro_time = 0;
    static int bytes_to_read = 2;
    static int fifo_bytes = 0;

    static uint8_t tmp_data[7];
    static int16_t gyro[3];
    static int16_t accel[3];

    static constexpr int64_t kGyroPrescale =
        16777216 * (35e-3 * 3.141592 / 180.0) / (1.0 / 32.8 * 3.141592 / 180.0);
    static constexpr int64_t kAccelPrescale = 16777216 * 0.488e-3 / (16.0 / 32767);

    if (mini_i2c_read_reg_get_result(tmp_data, bytes_to_read) == ESP_OK) {
        if (bytes_to_read == 2) {
            fifo_bytes = tmp_data[0] | ((tmp_data[1] & 0x03) << 8);

            if (fifo_bytes > 0) {
                bytes_to_read = 7;
            } else {
                bytes_to_read = 2;
            }

            if (fifo_bytes > 256) {
                while (1)
                    ;
            }
        } else {
            uint8_t tag = (tmp_data[0] >> 3);
            if (tag == REG_FIFO_DATA_OUT_TAG_VALUE_TAG_GYRO) {
                gyro[0] = (int16_t)((tmp_data[2] << 8) | tmp_data[1]) * kGyroPrescale / 16777216;
                gyro[1] = (int16_t)((tmp_data[4] << 8) | tmp_data[3]) * kGyroPrescale / 16777216;
                gyro[2] = (int16_t)((tmp_data[6] << 8) | tmp_data[5]) * kGyroPrescale / 16777216;
                have_gyro = true;
            } else if (tag == REG_FIFO_DATA_OUT_TAG_VALUE_TAG_ACCEL) {
                accel[0] = (int16_t)((tmp_data[2] << 8) | tmp_data[1]) * kAccelPrescale / 16777216;
                accel[1] = (int16_t)((tmp_data[4] << 8) | tmp_data[3]) * kAccelPrescale / 16777216;
                accel[2] = (int16_t)((tmp_data[6] << 8) | tmp_data[5]) * kAccelPrescale / 16777216;
                have_accel = true;
            }
            if (have_gyro) {
                gctx.gyro_ring->Push((time - prev_gyro_time) * 1000, gyro[0], gyro[1], gyro[2],
                                     accel[0], accel[1], accel[2], have_accel ? kFlagHaveAccel : 0);
                have_gyro = false;
                have_accel = false;
                pipeline_reset = false;
                prev_gyro_time = time;
            }
            fifo_bytes--;
            if (fifo_bytes == 0) {
                bytes_to_read = 2;
            }
        }
    } else {
        mini_i2c_hw_fsm_reset();
    }
    if (bytes_to_read == 2) {
        ESP_ERROR_CHECK(mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_STATUS1, 2,
                                                   gyro_i2c_cb, nullptr));
    } else {
        ESP_ERROR_CHECK(mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_DATA_OUT_TAG, 7,
                                                   gyro_i2c_cb, nullptr));
    }
}

bool probe_lsm6(uint8_t dev_adr) {
    static uint8_t data[1];
    if (mini_i2c_read_reg_sync(dev_adr, REG_WHO_AM_I, data, 1) != ESP_OK) {
        return false;
    }
    return data[0] == 0x6b;
}

void gyro_lsm6_task(void* params) {
    mini_i2c_set_timing(600000);
    mini_i2c_double_stop_timing();
    mini_i2c_double_stop_timing();

    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CTRL3_C, (3 << REG_CTRL3_C_BIT_SW_RESET));

    vTaskDelay(5);

    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CTRL9_XL, (3 << REG_CTRL9_XL_BIT_I3C_DISABLE));

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

    ESP_ERROR_CHECK(
        mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_STATUS1, 2, gyro_i2c_cb, nullptr));

    vTaskDelete(NULL);
}