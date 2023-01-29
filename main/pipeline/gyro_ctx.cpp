#include "gyro_ctx.hpp"

#include <cstring>

#include <esp_log.h>

BufQueue::BufQueue(size_t block_count, size_t block_size)
    : block_count(block_count), block_size(block_size) {
    data = (uint8_t*)malloc(block_size * block_count);
    alloc_queue = xQueueCreate(block_count, sizeof(Descriptor));
    desc_queue = xQueueCreate(block_count, sizeof(Descriptor));
    for (size_t i = 0; i < block_count; ++i) {
        Descriptor desc{.ptr = data + i * block_size, .size1 = 0, .size2 = 0, .dt = 0};
        xQueueSendToBack(alloc_queue, (void*)&desc, portMAX_DELAY);
    }
}

BufQueue::~BufQueue() {
    vQueueDelete(desc_queue);
    vQueueDelete(alloc_queue);
    ::free(data);
}

bool IRAM_ATTR BufQueue::alloc(Descriptor* desc) {
    return xQueueReceiveFromISR(alloc_queue, desc, nullptr) == pdTRUE;
}

bool IRAM_ATTR BufQueue::free(Descriptor* desc) {
    return xQueueSendToBackFromISR(alloc_queue, desc, nullptr) == pdTRUE;
}

bool IRAM_ATTR BufQueue::pop(Descriptor* desc) {
    return xQueueReceiveFromISR(desc_queue, desc, nullptr) == pdTRUE;
}

bool IRAM_ATTR BufQueue::push(Descriptor* desc) {
    return xQueueSendToBackFromISR(desc_queue, desc, nullptr) == pdTRUE;
}

static bool IRAM_ATTR gyro_flush(GyroCtx* ctx, bool gyro) {
    bool full{};
    if (gyro) full = ctx->desc.size1 >= ctx->gyr_block * 6;
    if (!gyro) full = ctx->desc.size2 >= ctx->acc_block * 6;
    full |= ctx->desc.ptr == nullptr;
    if (full) {
        if (ctx->desc.ptr != nullptr && !ctx->queue->push(&ctx->desc)) {
            ctx->queue->free(&ctx->desc);
        }
        ctx->desc.ptr = nullptr;
        if (!ctx->queue->alloc(&ctx->desc)) {
            ctx->desc.ptr = nullptr;
            return false;
        }
        ctx->desc.size1 = 0;
        ctx->desc.size2 = 0;
        ctx->desc.dt = 0;
    }
    return true;
}

static void IRAM_ATTR gyro_cb(int16_t* gyr, uint32_t dt, void* _ctx) {
    GyroCtx* ctx = (GyroCtx*)_ctx;
    if (!gyro_flush(ctx, true)) {
        ++ctx->drops;
        return;
    }
    if (ctx->desc.size1 >= ctx->gyr_block * 6) {
        ++ctx->drops;
        return;
    }
    memcpy(ctx->desc.ptr + ctx->desc.size1, gyr, 6);
    ctx->desc.size1 += 6;
    ctx->desc.dt += dt;
}

static void IRAM_ATTR accel_cb(int16_t* accel, void* _ctx) {
    GyroCtx* ctx = (GyroCtx*)_ctx;
    if (!gyro_flush(ctx, false)) {
        ++ctx->drops;
        return;
    }
    if (ctx->desc.size1 == 0) {
        ++ctx->drops;
        return;
    }
    if (ctx->desc.size2 >= ctx->acc_block * 6) {
        ++ctx->drops;
        return;
    }
    memcpy(ctx->desc.ptr + 6 * ctx->gyr_block + ctx->desc.size2, accel, 6);
    ctx->desc.size2 += 6;
}

bool gyro_ctx_init(GyroCtx* ctx, GyroHal* hal) {
    int timeout{1000};
    while (!hal->ready) {
        if (!timeout--) {
            ESP_LOGE("gyro_ctx", "gyro timeout");
            return false;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ctx->hal = hal;
    ctx->acc_block = 1;
    ctx->gyr_block = ctx->acc_block * hal->accel_div;
    {
        for (int a = 0; a < 1000; ++a) {
            int g = a * hal->accel_div;
            int final_a_div = -1;
            int final_g_div = -1;
            if (hal->gyro_sr / 4 <= g && g <= hal->gyro_sr / 2) {
                // ESP_LOGI("gyro_ctx", "blk %d/%d", a, g);
                float blk_freq = hal->gyro_sr / g;
                for (int a_div = 1; a_div <= a; ++a_div) {
                    if (a % a_div == 0) {
                        float a_final_sr = blk_freq * a / a_div;
                        if (7 <= a_final_sr && a_final_sr <= 13) {
                            // ESP_LOGI("gyro_ctx", "  sr a %f", a_final_sr);
                            final_a_div = a_div;
                        }
                    }
                }
                for (int g_div = 1; g_div <= g; ++g_div) {
                    if (g % g_div == 0) {
                        float g_final_sr = blk_freq * g / g_div;
                        if (490 <= g_final_sr && g_final_sr <= 590) {
                            // ESP_LOGI("gyro_ctx", "  sr g %f", g_final_sr);
                            final_g_div = g_div;
                        }
                    }
                }
                if (final_a_div > 0 && final_g_div > 0) {
                    ctx->acc_block = a;
                    ctx->gyr_block = g;
                    ctx->acc_div = final_a_div;
                    ctx->gyr_div = final_g_div;
                }
            }
        }
    }

    ESP_LOGI("gyro_ctx", "blk size: %d/%d -> %d/%d", ctx->gyr_block, ctx->acc_block,
             ctx->gyr_block / ctx->gyr_div, ctx->acc_block / ctx->acc_div);
    ESP_LOGI("gyro_ctx", "sr      : %f/%f", hal->gyro_sr / ctx->gyr_div, hal->gyro_sr / hal->accel_div / ctx->acc_div);
    ctx->queue = new BufQueue(hal->gyro_sr / ctx->gyr_block, (ctx->gyr_block + ctx->acc_block) * 6);
    ctx->queue->alloc(&ctx->desc);
    hal->cb_ctx = ctx;
    hal->gyro_cb = gyro_cb;
    hal->accel_cb = accel_cb;

    return true;
}