#include <stdio.h>
#include <driver/i2c.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>

#include "bus/bus_i2c.h"
#include "gyro/gyro.h"

static const char *TAG = "main";

void gyro_comsumer_task(void *params_pvoid)
{
    gyro_task_params *params = (gyro_task_params *)params_pvoid;

    for (int i = 0;; ++i)
    {
        gyro_sample_message msg;
        xQueueReceive(params->sample_queue, &msg, portMAX_DELAY);
        if (i % 100 == 0)
        {
            ESP_LOGI("gyro_consumer", "ts = %lld, gZ = %d", msg.timestamp, msg.gyro_z);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    QueueHandle_t sample_queue = xQueueCreate(GYRO_MAX_QUEUE_LENGTH, sizeof(gyro_sample_message));

    gyro_task_params *gyro_params = malloc(sizeof(gyro_task_params));
    gyro_params->sample_queue = sample_queue;
    xTaskCreate(gyro_mpu6050_task, "gyro-task", 4096, gyro_params, configMAX_PRIORITIES - 1, NULL);

    xTaskCreate(gyro_comsumer_task, "gyro-comsumer", 4096, gyro_params, configMAX_PRIORITIES - 1, NULL);
}