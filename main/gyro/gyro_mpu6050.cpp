// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_mpu6050.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>
#include <driver/timer.h>

#include "bus/bus_i2c.h"
}

#include "global_context.hpp"
#include "filters/gyro_ring.hpp"

static const char *TAG = "gyro_mpu";

#define DEV_ADDR 0x68

#define REG_SMPRT_DIV 0x19

#define REG_CONFIG 0x1a
#define REG_CONFIG_BIT_DLPF_CFG_0 0
#define REG_CONFIG_VALUE_DLPF_188HZ 1

#define REG_GYRO_CONFIG 0x1b
#define REG_GYRO_CONFIG_BIT_FS_SEL_0 3
#define REG_GYRO_CONFIG_VALUE_FS_250_DPS 0
#define REG_GYRO_CONFIG_VALUE_FS_500_DPS 1
#define REG_GYRO_CONFIG_VALUE_FS_1000_DPS 2
#define REG_GYRO_CONFIG_VALUE_FS_2000_DPS 3

#define REG_ACCEL_CONFIG 0x1c
#define REG_ACCEL_CONFIG_BIT_FS_SEL_0 3
#define REG_ACCEL_CONFIG_VALUE_FS_2_G 0
#define REG_ACCEL_CONFIG_VALUE_FS_4_G 1
#define REG_ACCEL_CONFIG_VALUE_FS_8_G 2
#define REG_ACCEL_CONFIG_VALUE_FS_16_G 3

#define REG_FIFO_EN 0x23
#define REG_FIFO_EN_MASK_TEMP (1 << 7)
#define REG_FIFO_EN_MASK_GYRO ((1 << 6) | (1 << 5) | (1 << 4))
#define REG_FIFO_EN_MASK_ACCEL (1 << 3)

#define REG_INT_STATUS 0x3a
#define REG_INT_STATUS_MASK_FIFO_OFLOW (1 << 4)
#define REG_INT_STATUS_MASK_DATA_RDY (1 << 0)

#define REG_USER_CONTROL 0x6a
#define REG_USER_CONTROL_MASK_FIFO_EN (1 << 6)
#define REG_USER_CONTROL_MASK_FIFO_RESET (1 << 2)
#define REG_USER_CONTROL_MASK_SIG_COND_RESET (1 << 0)

#define REG_PWR_MGMT_1 0x6b
#define REG_PWR_MGMT_1_MASK_RESET (1 << 7)
#define REG_PWR_MGMT_1_MASK_SLEEP (1 << 6)
#define REG_PWR_MGMT_1_BIT_CLKSEL_0 0
#define REG_PWR_MGMT_1_VALUE_CLKSEL_ZGYRO 3

#define REG_FIFO_COUNT_H 0x72
#define REG_FIFO_COUNT_L 0x73
#define REG_FIFO_RW 0x74

#define REG_WHO_AM_I 0x75

#define FIFO_SAMPLE_SIZE 12

#define TIMER_DIVIDER (16)  //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

static bool IRAM_ATTR gyro_timer_cb(void *args) {
    static uint64_t time = 0;
    static uint8_t tmp_data[FIFO_SAMPLE_SIZE];

    BaseType_t high_task_awoken = pdFALSE;

    i2c_register_read(DEV_ADDR, REG_FIFO_COUNT_H, tmp_data, 2);
    const int fifo_bytes = tmp_data[1] | (tmp_data[0] << 8);

    if (fifo_bytes > 0) {
        i2c_register_read(DEV_ADDR, REG_FIFO_RW, tmp_data, FIFO_SAMPLE_SIZE);
        gctx.gyro_ring->Push(time, (int16_t)((tmp_data[6] << 8) | tmp_data[7]),
                             (int16_t)((tmp_data[8] << 8) | tmp_data[9]),
                             (int16_t)((tmp_data[10] << 8) | tmp_data[11]),
                             (int16_t)((tmp_data[0] << 8) | tmp_data[1]),
                             (int16_t)((tmp_data[2] << 8) | tmp_data[3]),
                             (int16_t)((tmp_data[4] << 8) | tmp_data[5]), kFlagHaveAccel);
    }

    if (fifo_bytes > 900) {
        while (1)
            ;
    }
    time += 900;
    return high_task_awoken;
}

void gyro_mpu6050_task(void *params_pvoid) {
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_register_read(DEV_ADDR, REG_WHO_AM_I, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%X", data[0]);
    if (data[0] != 0x68) {
        ESP_LOGI(TAG, "Wrong WHO_AM_I value!");
        return;
    }

    /* Reset */
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1, REG_PWR_MGMT_1_MASK_RESET));
    ESP_LOGI(TAG, "IMU reset");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1, 0));
    ESP_LOGI(TAG, "IMU wake up");

    ESP_ERROR_CHECK(
        i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1,
                                REG_PWR_MGMT_1_VALUE_CLKSEL_ZGYRO << REG_PWR_MGMT_1_BIT_CLKSEL_0));
    ESP_LOGI(TAG, "IMU change clock");

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_CONFIG, 0 << REG_CONFIG_BIT_DLPF_CFG_0));
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_SMPRT_DIV, 7));
    ESP_ERROR_CHECK(
        i2c_register_write_byte(DEV_ADDR, REG_GYRO_CONFIG,
                                REG_GYRO_CONFIG_VALUE_FS_1000_DPS << REG_GYRO_CONFIG_BIT_FS_SEL_0));
    ESP_ERROR_CHECK(
        i2c_register_write_byte(DEV_ADDR, REG_ACCEL_CONFIG,
                                REG_ACCEL_CONFIG_VALUE_FS_16_G << REG_ACCEL_CONFIG_BIT_FS_SEL_0));

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_FIFO_EN,
                                            REG_FIFO_EN_MASK_ACCEL | REG_FIFO_EN_MASK_GYRO));
    ESP_ERROR_CHECK(
        i2c_register_write_byte(DEV_ADDR, REG_USER_CONTROL, REG_USER_CONTROL_MASK_FIFO_EN));

    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 9e-4 * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, gyro_timer_cb, NULL, ESP_INTR_FLAG_IRAM);

    timer_start(TIMER_GROUP_0, TIMER_0);

    vTaskDelete(NULL);
}