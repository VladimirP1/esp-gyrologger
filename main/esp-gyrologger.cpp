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

#include "misc/misc.hpp"
#include "misc/battery.hpp"
#include "wifi/http.hpp"
#include "wifi/wifi.hpp"
#include "wifi/cam_control.hpp"
#include "gyro/gyro.hpp"
#include "pipeline/gyro_ctx.hpp"
#include "pipeline/logger.hpp"
#include "hal/fs.hpp"
#include "storage/settings.hpp"

#include "global_context.hpp"
#include "bus/aux_i2c.hpp"
// #include "display.hpp"

#include "http/server.hpp"

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

    gctx.aux_i2c_queue = xQueueCreate(1, sizeof(aux_i2c_msg_t));
    gctx.settings_manager = new SettingsManager();
    gctx.gyro_hal = new GyroHal();
    gctx.gyro_ctx = new GyroCtx();

    // gctx.settings_manager->Set("wifi_dbm", 10);

#if EXPERIMENTAL_BATTERY
    xTaskCreate(battery_task, "battery_task", 3084, NULL, configMAX_PRIORITIES - 4, NULL);
#endif

    wifi_init();
    // display_setup();

    do {
        auto &sm = gctx.settings_manager;
        int mosi = sm->Get("sd_mosi");
        int miso = sm->Get("sd_miso");
        int sck = sm->Get("sd_sck");
        int cs = sm->Get("sd_cs");
        FsSettings fs_settings{.external_sd = mosi != -1 && miso != -1 && sck != -1 && cs != -1,
                               .pin_mosi = mosi,
                               .pin_miso = miso,
                               .pin_clk = sck,
                               .pin_cs = cs};
        if (fs_init(&fs_settings)) {
            break;
        }
        fs_settings.external_sd = false;
        if (fs_init(&fs_settings)) {
            break;
        }
    } while (0);

    do {
        int sda_pin = gctx.settings_manager->Get("sda_pin");
        int scl_pin = gctx.settings_manager->Get("scl_pin");
        if (sda_pin >= 0 && scl_pin >= 0) {
            if (!gyro_hal_init(gctx.gyro_hal, 5, 6)) {
                break;
            }
            if (!gyro_ctx_init(gctx.gyro_ctx, gctx.gyro_hal)) {
                break;
            }
            ESP_LOGI(TAG, "%s ready!", gctx.gyro_hal->gyro_type);

            gctx.logger_control.mutex = xSemaphoreCreateMutex();
            gctx.logger_control.accel_raw_mtx = xSemaphoreCreateMutex();

            xTaskCreate(logger_task, "logger", 3084, NULL, configMAX_PRIORITIES - 3, NULL);
        } else {
            ESP_LOGW(TAG, "Please assign i2c gpio pins!");
        }
    } while (0);

    if (gctx.settings_manager->Get("led_type") < 0.5) {
        xTaskCreate(led_task, "led-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    } else {
        xTaskCreate(led_strip_task, "led-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    }
    xTaskCreate(button_task, "button-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(cam_control_task, "cam-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    // xTaskCreate(display_task, "display-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);

    xTaskCreate(server_task, "server-task", 8192, NULL, configMAX_PRIORITIES - 4, NULL);

    // http_init();
}

extern "C" {
void app_main(void) { app_main_cpp(); }
}