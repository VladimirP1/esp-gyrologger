#include "interpolate.h"

#include "global_context.h"

#include <esp_log.h>

#define TAG "interpolator"

static uint64_t smooth_timestamp(uint64_t rough_timestamp) {
    static int gyro_counter = 0;
    static uint64_t gyro_counter_reset_ts = 0;
    static uint64_t avg_sample_interval_ns = 602410;

    static uint64_t timestamp = 0;
    static uint16_t timestamp_frac = 0;

    timestamp += avg_sample_interval_ns / 1000;
    timestamp_frac += avg_sample_interval_ns % 1000;
    if (timestamp_frac >= 1000) {
        timestamp_frac -= 1000;
        timestamp += 1;
    }
    ++gyro_counter;

    if (gyro_counter > 5000) {
        avg_sample_interval_ns = (rough_timestamp - gyro_counter_reset_ts) * 1000 / gyro_counter;
        gyro_counter = 0;
        gyro_counter_reset_ts = rough_timestamp;
    }
}

void interpolator_task(void* params) {
    const int sample_interval = 1750;
    uint64_t current_time = 0;

    bool write_idx = 0;
    gyro_sample_message ring[2];
    while (true) {
        xQueueReceive(gctx.gyro_raw_queue, &ring[write_idx], portMAX_DELAY);

        write_idx = !write_idx;

        if (ring[!write_idx].fifo_backlog > 1000) {
            ESP_LOGE(TAG, "fifo overrun %d", ring[!write_idx].fifo_backlog);
            abort();
        }

        // advance current time if possible
        while (current_time <= ring[!write_idx].timestamp) {
            // ESP_LOGI(TAG, "current time=%llu, msg time=%llu", current_time,
            // ring[!write_idx].timestamp);

            gyro_sample_message* a = &ring[write_idx];
            gyro_sample_message* b = &ring[!write_idx];

            // smooth the timestamps
            b->timestamp = smooth_timestamp(b->timestamp);

            // calculate weights
            int64_t wa = current_time - a->timestamp;
            int64_t wb = b->timestamp - current_time;
            int64_t denom = wa + wb;

            if (wa < 0 || wb < 0) {
                ESP_LOGW(TAG, "bad sample weights");
                continue;
            }

            // interpolate
            gyro_sample_message msg = *a;
            msg.timestamp = current_time;
            msg.gyro_x = (a->gyro_x * wa + b->gyro_x * wb) / denom;
            msg.gyro_y = (a->gyro_y * wa + b->gyro_y * wb) / denom;
            msg.gyro_z = (a->gyro_z * wa + b->gyro_z * wb) / denom;

            msg.accel_x = (a->accel_x * wa + b->accel_x * wb) / denom;
            msg.accel_y = (a->accel_y * wa + b->accel_y * wb) / denom;
            msg.accel_z = (a->accel_z * wa + b->accel_z * wb) / denom;

            xQueueSend(gctx.gyro_interp_queue, &msg, portMAX_DELAY);

            current_time += sample_interval;
        }
    }
}