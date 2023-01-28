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
#include "pipeline/gyro_ctx.hpp"
#include "bus/aux_i2c.hpp"
#include "global_context.hpp"

static const char* TAG = "main";

static void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

GyroHal gyro_hal{};
GyroCtx gyro_ctx{};

void app_main_cpp(void) {
    ESP_LOGI(TAG, "heap %u", esp_get_free_heap_size());

    nvs_init();
    gctx.aux_i2c_queue = xQueueCreate(1, sizeof(aux_i2c_msg_t));

    if (gyro_hal_init(&gyro_hal, 5, 6)) {
        if (gyro_ctx_init(&gyro_ctx, &gyro_hal)) {
            ESP_LOGI(TAG, "%s ready!", gyro_hal.gyro_type);
        }
    }

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