#pragma once

#include <stdint.h>
#include <stdbool.h>

#define GYRO_MAX_QUEUE_LENGTH 2048

typedef struct
{
    uint64_t timestamp;
    int gyro_x, gyro_y, gyro_z;
    int accel_x, accel_y, accel_z;
    uint32_t smpl_interval_ns;
    uint32_t flags;
} gyro_sample_message;

typedef enum {
    GYRO_SAMPLE_NEW_ACCEL_DATA = 1,
} gyro_sample_flags_t;