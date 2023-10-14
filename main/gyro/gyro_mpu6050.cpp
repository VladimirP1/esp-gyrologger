// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gyro_mpu6050.hpp"

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

static const char *TAG = "gyro_mpu";

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

enum {IMU_MPU6000, IMU_MPU6886};
static int fifo_sample_size{};
static int mpu_imu_type{};

static void IRAM_ATTR gyro_i2c_fifo_bytes_cb(void *arg);

static void IRAM_ATTR gyro_i2c_cb_mpu(void *arg) {
    if (gctx.terminate_for_update) {
        return;
    }

    static uint8_t tmp_data[14];
    static uint64_t prev_time = 0;
    uint64_t time = esp_timer_get_time();
    if (mini_i2c_read_reg_get_result(tmp_data, fifo_sample_size) == ESP_OK) {
        int16_t gyr[3];
        int16_t acc[3];
        if (mpu_imu_type == IMU_MPU6000) {
            gyr[0] = (int16_t)((tmp_data[6] << 8) | tmp_data[7]);
            gyr[1] = (int16_t)((tmp_data[8] << 8) | tmp_data[9]);
            gyr[2] = (int16_t)((tmp_data[10] << 8) | tmp_data[11]);
            acc[0] = (int16_t)((tmp_data[0] << 8) | tmp_data[1]);
            acc[1] = (int16_t)((tmp_data[2] << 8) | tmp_data[3]);
            acc[2] = (int16_t)((tmp_data[4] << 8) | tmp_data[5]);
        } else { // IMU_MPU6886
            gyr[0] = (int16_t)((tmp_data[8] << 8) | tmp_data[9]); 
            gyr[1] = (int16_t)((tmp_data[10] << 8) | tmp_data[11]);
            gyr[2] = (int16_t)((tmp_data[12] << 8) | tmp_data[13]);
            acc[0] = (int16_t)((tmp_data[0] << 8) | tmp_data[1]);
            acc[1] = (int16_t)((tmp_data[2] << 8) | tmp_data[3]);
            acc[2] = (int16_t)((tmp_data[4] << 8) | tmp_data[5]);
        }
        gctx.gyro_ring->Push((time - prev_time) * 1000, gyr[0], gyr[1], gyr[2], acc[0], acc[1],
                             acc[2], kFlagHaveAccel);
        prev_time = time;
    } else {
        /* ignore */
    }

    proc_aux_i2c();
    mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_COUNT_H, 2, gyro_i2c_fifo_bytes_cb,
                               nullptr);
}


static void IRAM_ATTR gyro_i2c_fifo_bytes_cb(void *arg) {
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
        /* ignore */
    }

    if (fifo_bytes >= fifo_sample_size) {
        mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_RW, fifo_sample_size,
                                   gyro_i2c_cb_mpu, NULL);
    } else {
        mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_COUNT_H, 2, gyro_i2c_fifo_bytes_cb,
                                   nullptr);
    }
}

bool probe_mpu6050(uint8_t dev_adr) {
    uint8_t data[1];
    if (mini_i2c_read_reg_sync(dev_adr, REG_WHO_AM_I, data, 1) != ESP_OK) {
        return false;
    }
    return data[0] == 0x68 || data[0] == 0x19;
}

void gyro_mpu6050_task(void *params_pvoid) {
    /* This is needed for 2k sample rate */
    mini_i2c_set_timing(500000);

    /* Mpu6050 needs larger delays between transfers */
    mini_i2c_double_stop_timing();
    mini_i2c_double_stop_timing();

    /* Detect imu type */
    uint8_t data[1];
    mini_i2c_read_reg_sync(gctx.gyro_i2c_adr, REG_WHO_AM_I, data, 1);
    if (data[0] == 0x68) {
        ESP_LOGI(TAG, "IMU is MPU6000");
        mpu_imu_type = IMU_MPU6000;
        fifo_sample_size = 12;
        gctx.gyro_sr = 2000.0;
        gctx.accel_sr = 2000.0;
    } else if (data[0] == 0x19) {
        ESP_LOGI(TAG, "IMU is MPU6886");
        mpu_imu_type = IMU_MPU6886;
        fifo_sample_size = 14;
        gctx.gyro_sr = 1000.0;
        gctx.accel_sr = 1000.0;
    } else {
        ESP_LOGE(TAG, "bad whoami");
        vTaskDelete(NULL);
    }

    /* Reset */
    ESP_ERROR_CHECK(
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_PWR_MGMT_1, REG_PWR_MGMT_1_MASK_RESET));
    ESP_LOGI(TAG, "IMU reset");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_PWR_MGMT_1, 0));
    ESP_LOGI(TAG, "IMU wake up");

    ESP_ERROR_CHECK(
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_PWR_MGMT_1,
                                REG_PWR_MGMT_1_VALUE_CLKSEL_ZGYRO << REG_PWR_MGMT_1_BIT_CLKSEL_0));
    ESP_LOGI(TAG, "IMU change clock");

    if (mpu_imu_type == IMU_MPU6000) {
        ESP_ERROR_CHECK(
            mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CONFIG, 0 << REG_CONFIG_BIT_DLPF_CFG_0));
        ESP_ERROR_CHECK(mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_SMPRT_DIV, 3));
    } else if (mpu_imu_type == IMU_MPU6886) {
        ESP_ERROR_CHECK(
            mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_CONFIG, 1 << REG_CONFIG_BIT_DLPF_CFG_0));
        ESP_ERROR_CHECK(mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_SMPRT_DIV, 0));
    }

    ESP_ERROR_CHECK(
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_GYRO_CONFIG,
                                REG_GYRO_CONFIG_VALUE_FS_1000_DPS << REG_GYRO_CONFIG_BIT_FS_SEL_0));
    ESP_ERROR_CHECK(
        mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_ACCEL_CONFIG,
                                REG_ACCEL_CONFIG_VALUE_FS_16_G << REG_ACCEL_CONFIG_BIT_FS_SEL_0));

    ESP_ERROR_CHECK(mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_FIFO_EN,
                                            REG_FIFO_EN_MASK_ACCEL | REG_FIFO_EN_MASK_GYRO));
    ESP_ERROR_CHECK(mini_i2c_write_reg_sync(gctx.gyro_i2c_adr, REG_USER_CONTROL,
                                            REG_USER_CONTROL_MASK_FIFO_EN));

    mini_i2c_read_reg_callback(gctx.gyro_i2c_adr, REG_FIFO_COUNT_H, 2, gyro_i2c_fifo_bytes_cb,
                               nullptr);

    vTaskDelete(NULL);
}