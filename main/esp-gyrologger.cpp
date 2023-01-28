// SPDX-License-Identifier: LGPL-2.1-or-later

extern "C" {
#include "bus/mini_i2c.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_console.h>
#include <nvs_flash.h>

#include <string.h>
#include <stdio.h>
}

#include "gyro/gyro.hpp"
#include "global_context.hpp"
#include "bus/aux_i2c.hpp"
#include "filters/pt_filter.hpp"

static const char* TAG = "main";

static void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

struct Descriptor {
    void* ptr{};
    uint32_t size1{};
    uint32_t size2{};
    uint32_t dt{};
};

struct BufQueue {
    BufQueue(size_t block_count, size_t block_size)
        : block_count(block_count), block_size(block_size) {
        data = (uint8_t*)malloc(block_size * block_count);
        alloc_queue = xQueueCreate(block_count, sizeof(Descriptor));
        desc_queue = xQueueCreate(block_count, sizeof(Descriptor));
        for (size_t i = 0; i < block_count; ++i) {
            Descriptor desc{.ptr = data + i * block_size, .size1 = 0, .size2 = 0, .dt = 0};
            xQueueSendToBack(alloc_queue, (void*)&desc, portMAX_DELAY);
        }
    }

    ~BufQueue() {
        vQueueDelete(desc_queue);
        vQueueDelete(alloc_queue);
        ::free(data);
    }
    BufQueue(const BufQueue&) = delete;
    BufQueue(BufQueue&&) = delete;
    BufQueue& operator=(const BufQueue&) = delete;
    BufQueue& operator=(BufQueue&&) = delete;

    bool alloc(Descriptor* desc);
    bool free(Descriptor* desc);
    bool pop(Descriptor* desc);
    bool push(Descriptor* desc);

    size_t block_count{};
    size_t block_size{};

   private:
    uint8_t* data{};
    QueueHandle_t alloc_queue{};
    QueueHandle_t desc_queue{};
};

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

struct GyroCtx {
    GyroHal* hal{};
    BufQueue* queue{};
    Descriptor desc{};
    size_t gyr_block{};
    size_t acc_block{};
    size_t gyr_cnt{};
    size_t acc_cnt{};
    size_t drops{};
};

GyroHal gyro_hal{};
GyroCtx gyro_ctx{};

bool IRAM_ATTR gyro_flush(GyroCtx* ctx, bool gyro) {
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

void IRAM_ATTR gyro_cb(int16_t* gyr, uint32_t dt, void* _ctx) {
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

void IRAM_ATTR accel_cb(int16_t* accel, void* _ctx) {
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

void app_main_cpp(void) {
    ESP_LOGI(TAG, "heap %u", esp_get_free_heap_size());

    nvs_init();

    int sda_pin = 5;
    int scl_pin = 6;
    if (sda_pin >= 0 && scl_pin >= 0) {
        ESP_ERROR_CHECK(mini_i2c_init(sda_pin, scl_pin, 400000));
        gctx.aux_i2c_queue = xQueueCreate(1, sizeof(aux_i2c_msg_t));
        gyro_probe_and_start_task(&gyro_hal);
    } else {
        ESP_LOGW(TAG, "Please assign i2c gpio pins!");
    }

    while (!gyro_hal.ready) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    {
        gyro_ctx.hal = &gyro_hal;
        gyro_ctx.acc_block = 1;
        while ((gyro_ctx.acc_block + 1) * gyro_hal.accel_div <= 256) ++gyro_ctx.acc_block;
        gyro_ctx.gyr_block = gyro_ctx.acc_block * gyro_hal.accel_div;
        ESP_LOGI("main", "blk size: gyro: %d, acc: %d", gyro_ctx.gyr_block, gyro_ctx.acc_block);
        gyro_ctx.queue = new BufQueue(gyro_hal.gyro_sr / gyro_ctx.gyr_block,
                                      (gyro_ctx.gyr_block + gyro_ctx.acc_block) * 6);
        gyro_ctx.queue->alloc(&gyro_ctx.desc);
        gyro_hal.cb_ctx = &gyro_ctx;
        gyro_hal.gyro_cb = gyro_cb;
        gyro_hal.accel_cb = accel_cb;
    }

    ESP_LOGI(TAG, "%s ready!", gyro_hal.gyro_type);

    while (1) {
        for (int i = 0; i < 100; ++i) {
            Descriptor desc{};
            while (!gyro_ctx.queue->pop(&desc)) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            gyro_ctx.queue->free(&desc);
            ESP_LOGI(TAG, "%d %d %d %u", desc.size1, desc.size2, desc.dt, esp_get_free_heap_size());
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

extern "C" {
void app_main(void) { app_main_cpp(); }
}