// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_icm42688.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>
#include <esp_timer.h>

#include "bus/mini_i2c.h"
}

#include "global_context.hpp"
#include "filters/gyro_ring.hpp"
#include "bus/aux_i2c.hpp"

static const char *TAG = "gyro_icm";

#define REG_WHO_AM_I 0x75
#define REG_BANK_SEL 0x76

// bank 0
#define REG_DEVICE_CONFIG 0x11
#define REG_FIFO_CONFIG 0x16
#define REG_FIFO_COUNTH 0x2e
#define REG_FIFO_COUNTL 0x2f
#define REG_FIFO_DATA 0x30
#define REG_PWR_MGMT0 0x4e
#define REG_GYRO_CONFIG0 0x4f
#define REG_ACCEL_CONFIG0 0x50
#define REG_GYRO_CONFIG1 0x51
#define REG_GYRO_ACCEL_CONFIG0 0x52
#define REG_ACCEL_CONFIG1 0x53
#define REG_FIFO_CONFIG1 0x5f

// bank 1
#define REG_SENSOR_CONFIG0 0x03

#define FIFO_SAMPLE_SIZE 16

static void IRAM_ATTR gyro_i2c_fifo_hdr_cb(void *arg);

static void IRAM_ATTR gyro_i2c_cb(void *arg) {
    if (gctx.terminate_for_update) {
        return;
    }

    static uint8_t tmp_data[FIFO_SAMPLE_SIZE];
    static uint64_t prev_time = 0;
    uint64_t time = esp_timer_get_time();
    if (mini_i2c_read_reg_get_result(tmp_data, FIFO_SAMPLE_SIZE) == ESP_OK) {
        gctx.gyro_ring->Push((time - prev_time) * 1000,
                             static_cast<int>((int16_t)((tmp_data[7] << 8) | tmp_data[8])),
                             static_cast<int>((int16_t)((tmp_data[9] << 8) | tmp_data[10])),
                             static_cast<int>((int16_t)((tmp_data[11] << 8) | tmp_data[12])),
                             static_cast<int>((int16_t)((tmp_data[1] << 8) | tmp_data[2])),
                             static_cast<int>((int16_t)((tmp_data[3] << 8) | tmp_data[4])),
                             static_cast<int>((int16_t)((tmp_data[5] << 8) | tmp_data[6])),
                             kFlagHaveAccel);
        prev_time = time;
    } else {
        mini_i2c_hw_fsm_reset();
    }

    mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_COUNTH, 2, gyro_i2c_fifo_hdr_cb,
                               nullptr);
}

static void IRAM_ATTR gyro_i2c_fifo_hdr_cb(void *arg) {
    if (gctx.terminate_for_update) {
        return;
    }

    int fifo_bytes = 0;
    uint8_t tmp_data[2];
    if (mini_i2c_read_reg_get_result(tmp_data, 2) == ESP_OK) {
        fifo_bytes = tmp_data[1] | (tmp_data[0] << 8);

        if (fifo_bytes > 900) {
            while (1)
                ;
        }
    } else {
        mini_i2c_hw_fsm_reset();
    }

    if (fifo_bytes > 0) {
        mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_DATA, FIFO_SAMPLE_SIZE, gyro_i2c_cb,
                                   NULL);
    } else {
        proc_aux_i2c();
        mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_COUNTH, 2, gyro_i2c_fifo_hdr_cb,
                                   nullptr);
    }
}

bool probe_icm42688(uint8_t dev_adr) {
    uint8_t data[1];
    if (mini_i2c_read_reg_sync(dev_adr, REG_WHO_AM_I, data, 1) != ESP_OK) {
        return false;
    }
    return data[0] == 0x47;
}

void gyro_icm42688_task(void *params_pvoid) {
    gctx.gyro_sr = 2000.0;
    gctx.accel_sr = 2000.0;

    mini_i2c_set_timing(900000);

    mini_i2c_double_stop_timing();

    // soft reset
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_BANK_SEL, 0);
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_DEVICE_CONFIG, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_DEVICE_CONFIG, 0);

    // bank 0
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_BANK_SEL, 0);
    // 6-axis low-noise mode
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_PWR_MGMT0, 0x1f);
    // 2k / 1000dps gyro
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYRO_CONFIG0, 0x25);
    // 2k / 16g accel
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_ACCEL_CONFIG0, 0x05);
    // lpf gyro 500hz, accel 50hz
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYRO_ACCEL_CONFIG0, 0x01);
    // stream-to-fifo mode
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CONFIG, 0x40);
    // enable fifo for gyro + accel
    mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_CONFIG1, 0x03);

    mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_COUNTH, 2, gyro_i2c_fifo_hdr_cb,
                               nullptr);

    vTaskDelete(nullptr);
}