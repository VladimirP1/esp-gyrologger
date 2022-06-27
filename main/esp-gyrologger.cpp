// SPDX-License-Identifier: LGPL-2.1-or-later

extern "C" {
#include "bus/bus_i2c.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_console.h>
#include <nvs_flash.h>

#include <string.h>
#include <stdio.h>

#include "storage/storage_fat.h"
#include "misc/misc.h"
}

#include "wifi/http.hpp"
#include "wifi/wifi.hpp"
#include "wifi/cam_control.hpp"
#include "gyro/gyro.hpp"
#include "logger/logger.hpp"
#include "filters/gyro_ring.hpp"

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

    wifi_init();

    ESP_ERROR_CHECK(storage_fat_init());

    ESP_ERROR_CHECK(i2c_master_init());

    gctx.logger_control.mutex = xSemaphoreCreateMutex();
    gctx.gyro_ring = new GyroRing();
    gctx.gyro_ring->Init(2048, kBlockSize, 1800);

    xTaskCreate(logger_task, "logger", 4096, NULL, configMAX_PRIORITIES - 2, NULL);
    // xTaskCreate(gyro_mpu6050_task, "gyro-task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    // xTaskCreate(gyro_lsm6_task, "gyro-task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(gyro_bmi160_task, "gyro-task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    // xTaskCreate(led_task, "led-task", 4096, NULL, configMAX_PRIORITIES - 3, NULL);
    // xTaskCreate(camera_task, "cam-task", 4096, NULL, configMAX_PRIORITIES - 3, NULL);

    http_init();
}

extern "C" {
void app_main(void) { app_main_cpp(); }
}