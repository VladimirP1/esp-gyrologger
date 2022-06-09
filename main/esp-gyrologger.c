#include <stdio.h>
#include <driver/i2c.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_console.h>
#include <nvs_flash.h>

#include "bus/bus_i2c.h"
#include "gyro/gyro.h"
#include "storage/storage_fat.h"
#include "logger/logger.h"
#include "wifi/wifi.h"
#include "wifi/http.h"

#include <string.h>

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(storage_fat_init());

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    QueueHandle_t sample_queue = xQueueCreate(GYRO_MAX_QUEUE_LENGTH, sizeof(gyro_sample_message));

    gyro_task_params *gyro_params = (gyro_task_params *)malloc(sizeof(gyro_task_params));
    gyro_params->sample_queue = sample_queue;
    xTaskCreate(gyro_mpu6050_task, "gyro-task", 4096, gyro_params, configMAX_PRIORITIES - 1, NULL);
    vTaskDelay(100);
    xTaskCreate(logger_task, "logger", 4096, gyro_params, configMAX_PRIORITIES - 2, NULL);

    // Console init
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif
    register_logger_cmd();

    wifi_init_softap();

    http_init();

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}