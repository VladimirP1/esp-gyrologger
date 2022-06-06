#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define GYRO_MAX_QUEUE_LENGTH 2048

typedef struct
{
    QueueHandle_t sample_queue;
} gyro_task_params;

typedef struct
{
    uint64_t timestamp;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t accel_x, accel_y, accel_z;
    uint16_t fifo_backlog;
} gyro_sample_message;