#include "interpolate.h"

#include "global_context.h"

#include <esp_log.h>

#define TAG "interpolator"

typedef struct {
    int gyro_counter;
    uint64_t gyro_counter_reset_ts;
    uint64_t avg_sample_interval_ns;

    uint64_t timestamp;
    uint16_t timestamp_frac;
} ts_smooth_data;

static void smooth_init(ts_smooth_data* state) {
    state->gyro_counter = 0;
    state->gyro_counter_reset_ts = 0;
    state->avg_sample_interval_ns = 1000;
    state->timestamp = 0;
    state->timestamp_frac = 0;
}

static uint64_t smooth_update(ts_smooth_data* state, uint64_t rough_timestamp) {
    state->timestamp += state->avg_sample_interval_ns / 1000;
    state->timestamp_frac += state->avg_sample_interval_ns % 1000;
    if (state->timestamp_frac >= 1000) {
        state->timestamp_frac -= 1000;
        state->timestamp += 1;
    }
    ++state->gyro_counter;

    if (state->gyro_counter > 10000) {
        state->avg_sample_interval_ns =
            (rough_timestamp - state->gyro_counter_reset_ts) * 1000 / state->gyro_counter;
        state->gyro_counter = 0;
        state->gyro_counter_reset_ts = rough_timestamp;
        ESP_LOGI(TAG, "avg_sample_interval_ns = %llu", state->avg_sample_interval_ns);
    }
    return state->timestamp;
}

void interpolator_task(void* params) {
    const int sample_interval = 300;
    uint64_t current_time = 0;
    ts_smooth_data smooth_state;
    smooth_init(&smooth_state);

    bool write_idx = 0;
    gyro_sample_message ring[2] = {{}, {}};

    while (true) {
        xQueueReceive(gctx.gyro_raw_queue, &ring[write_idx], portMAX_DELAY);

        write_idx = !write_idx;

        gyro_sample_message* a = &ring[write_idx];
        gyro_sample_message* b = &ring[!write_idx];

        // smooth the timestamps
        b->timestamp = smooth_update(&smooth_state, b->timestamp);

        // advance current time if possible
        while (current_time <= ring[!write_idx].timestamp) {
            // ESP_LOGI(TAG, "current time=%llu, msg time=%llu", current_time,
            // ring[!write_idx].timestamp);

            // calculate weights
            int64_t wa = current_time - a->timestamp;
            int64_t wb = b->timestamp - current_time;
            int64_t denom = wa + wb;

            if (wa < 0 || wb < 0) {
                ESP_LOGW(TAG, "bad sample weights");
                continue;
            }
            
            // interpolate
            gyro_sample_message msg = *b;
            msg.timestamp = current_time;
            msg.gyro_x = (a->gyro_x * wa + b->gyro_x * wb) / denom;
            msg.gyro_y = (a->gyro_y * wa + b->gyro_y * wb) / denom;
            msg.gyro_z = (a->gyro_z * wa + b->gyro_z * wb) / denom;

            msg.accel_x = (a->accel_x * wa + b->accel_x * wb) / denom;
            msg.accel_y = (a->accel_y * wa + b->accel_y * wb) / denom;
            msg.accel_z = (a->accel_z * wa + b->accel_z * wb) / denom;

            msg.smpl_interval_ns = smooth_state.avg_sample_interval_ns;

            xQueueSend(gctx.gyro_interp_queue, &msg, portMAX_DELAY);

            a->flags &= ~GYRO_SAMPLE_NEW_ACCEL_DATA;
            b->flags &= ~GYRO_SAMPLE_NEW_ACCEL_DATA;

            current_time += sample_interval;
        }
    }
}