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
#include "wifi/wifi.h"
#include "misc/misc.h"
}

#include "wifi/http.hpp"
#include "gyro/gyro.hpp"
#include "logger/logger.hpp"

#include "global_context.hpp"

static const char *TAG = "main";

void app_main_cpp(void) {
    wifi_init_softap();

    ESP_ERROR_CHECK(storage_fat_init());

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    gctx.logger_control.mutex = xSemaphoreCreateMutex();
    gctx.gyro_ring.Init(2048, kBlockSize, 1800);

    xTaskCreate(logger_task, "logger", 4096, NULL, configMAX_PRIORITIES - 2, NULL);
    // xTaskCreate(gyro_mpu6050_task, "gyro-task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    // xTaskCreate(gyro_lsm6_task, "gyro-task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(gyro_bmi160_task, "gyro-task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    // xTaskCreate(led_task, "led-task", 4096, NULL, configMAX_PRIORITIES - 3, NULL);

    http_init();

    // Console init
    //     esp_console_repl_t *repl = NULL;
    //     esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    // #if CONFIG_ESP_CONSOLE_UART
    //     esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    //     ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    // #elif CONFIG_ESP_CONSOLE_USB_CDC
    //     esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    //     ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
    // #elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    //     esp_console_dev_usb_serial_jtag_config_t usbjtag_config =
    //     ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    //     ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config,
    //     &repl));
    // #endif
    // register_logger_cmd();


    // ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

extern "C" {
void app_main(void) { app_main_cpp(); }
}