#include "gyro_mpu6050.h"
#include "gyro_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>

#include "bus/bus_i2c.h"

static const char *TAG = "gyro_mpu";

#define DEV_ADDR 0x68

#define REG_SMPRT_DIV 0x19

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

static bool gyro_enqueue_sample(gyro_task_params *params, uint8_t *sample, uint64_t timestamp)
{
    gyro_sample_message msg = {
        .timestamp = timestamp,
        .accel_x = (sample[0] << 8) | sample[1],
        .accel_y = (sample[2] << 8) | sample[3],
        .accel_z = (sample[4] << 8) | sample[5],
        .gyro_x = (sample[6] << 8) | sample[7],
        .gyro_y = (sample[8] << 8) | sample[9],
        .gyro_z = (sample[10] << 8) | sample[11]
    };

    return xQueueSendToBack(params->sample_queue, &msg, 0) != errQUEUE_FULL;
}

void gyro_mpu6050_task(void *params_pvoid)
{
    gyro_task_params *params = (gyro_task_params *)params_pvoid;

    uint8_t data[FIFO_SAMPLE_SIZE];
    ESP_ERROR_CHECK(i2c_register_read(DEV_ADDR, REG_WHO_AM_I, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%X", data[0]);
    if (data[0] != 0x68)
    {
        ESP_LOGI(TAG, "Wrong WHO_AM_I value!");
        return ESP_FAIL;
    }

    /* Reset */
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1, REG_PWR_MGMT_1_MASK_RESET));
    ESP_LOGI(TAG, "IMU reset");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1, 0));
    ESP_LOGI(TAG, "IMU wake up");

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1, REG_PWR_MGMT_1_VALUE_CLKSEL_ZGYRO << REG_PWR_MGMT_1_BIT_CLKSEL_0));
    ESP_LOGI(TAG, "IMU change clock");

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_SMPRT_DIV, 7));
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_GYRO_CONFIG, REG_GYRO_CONFIG_VALUE_FS_2000_DPS << REG_GYRO_CONFIG_BIT_FS_SEL_0));
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_ACCEL_CONFIG, REG_ACCEL_CONFIG_VALUE_FS_16_G << REG_ACCEL_CONFIG_BIT_FS_SEL_0));

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_FIFO_EN, REG_FIFO_EN_MASK_ACCEL | REG_FIFO_EN_MASK_GYRO));
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_USER_CONTROL, REG_USER_CONTROL_MASK_FIFO_EN));

    uint64_t samples_total = 0;
    for (;;)
    {
        ESP_ERROR_CHECK(i2c_register_read(DEV_ADDR, REG_INT_STATUS, data, 1));
        if (data[0] & REG_INT_STATUS_MASK_FIFO_OFLOW)
        {
            ESP_LOGI(TAG, "FIFO overflow!");
            while (true)
                ;
        }

        ESP_ERROR_CHECK(i2c_register_read(DEV_ADDR, REG_FIFO_COUNT_H, data, 2));
        const int fifo_bytes = data[1] | (data[0] << 8);
        for (int fifo_bytes_left = fifo_bytes; fifo_bytes_left > 0; fifo_bytes_left -= FIFO_SAMPLE_SIZE)
        {
            ESP_ERROR_CHECK(i2c_register_read(DEV_ADDR, REG_FIFO_RW, data, FIFO_SAMPLE_SIZE));
            gyro_enqueue_sample(params, data, samples_total * 1000ULL);
            ++samples_total;
        }

        // ESP_LOGI(TAG, "Read %d samples", fifo_bytes / FIFO_SAMPLE_SIZE);

        vTaskDelay(1);
    }
}