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
#include "hal/fs.hpp"
#include "bus/aux_i2c.hpp"
#include "global_context.hpp"

#include <fcntl.h>
#include <unistd.h>

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

    do {
        if (!gyro_hal_init(&gyro_hal, 5, 6)) {
            break;
        }
        if (!gyro_ctx_init(&gyro_ctx, &gyro_hal)) {
            break;
        }
        ESP_LOGI(TAG, "%s ready!", gyro_hal.gyro_type);
    } while (0);

    do {
        FsSettings fs_settings{
            .external_sd = true, .pin_mosi = 3, .pin_miso = 7, .pin_clk = 8, .pin_cs = 4};
        if (fs_init(&fs_settings)) {
            break;
        }
        fs_settings.external_sd = false;
        if (fs_init(&fs_settings)) {
            break;
        }
    } while (0);

    ESP_LOGI("main", "init done! free heap: %u", esp_get_free_heap_size());

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    int fd = open("/flash/test.txt", O_CREAT | O_TRUNC | O_WRONLY);

    const char s[] = "Hello, world!\n";
    write(fd, s, strlen(s));
    fsync(fd);

    // while (1) {
    //     for (int i = 0; i < 100; ++i) {
    //         Descriptor desc{};
    //         while (!gyro_ctx.queue->pop(&desc)) {
    //             vTaskDelay(10 / portTICK_PERIOD_MS);
    //         }
    //         gyro_ctx.queue->free(&desc);
    //         ESP_LOGI(TAG, "%d %d %d %u", desc.size1, desc.size2, desc.dt,
    //         esp_get_free_heap_size());
    //     }
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    // }
}

extern "C" {
void app_main(void) { app_main_cpp(); }
}