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

#include "storage/storage_fat.h"
}

#include "misc/misc.hpp"
#include "wifi/http.hpp"
#include "wifi/wifi.hpp"
#include "wifi/cam_control.hpp"
#include "gyro/gyro.hpp"
#include "logger/logger.hpp"
#include "filters/gyro_ring.hpp"
#include "storage/settings.hpp"

#include "global_context.hpp"

static const char *TAG = "main";

static void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void app_main_cpp(void) {
    nvs_init();

    gctx.settings_manager = new SettingsManager();

    wifi_init();

    ESP_ERROR_CHECK(storage_fat_init());

    ESP_ERROR_CHECK(mini_i2c_init(gctx.settings_manager->Get("sda_pin"),
                                  gctx.settings_manager->Get("scl_pin"), 400000));

    gctx.logger_control.mutex = xSemaphoreCreateMutex();
    gctx.gyro_ring = new GyroRing();
    gctx.gyro_ring->Init(3072, kBlockSize, 1800);

    xTaskCreate(logger_task, "logger", 3084, NULL, configMAX_PRIORITIES - 3, NULL);

    gyro_probe_and_start_task();

    xTaskCreate(led_task, "led-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(button_task, "button-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    // xTaskCreate(camera_task, "cam-task", 4096, NULL, configMAX_PRIORITIES - 3, NULL);

    http_init();
}

extern "C" {
void app_main(void) { app_main_cpp(); }
}