// SPDX-License-Identifier: LGPL-2.1-or-later

#include "storage_fat.h"

#include <esp_vfs.h>
#include <esp_littlefs.h>
#include <esp_system.h>
#include <esp_log.h>
#include <soc/timer_group_reg.h>
#include <hal/wdt_hal.h>

const char* base_path = "/spiflash";

static const char* TAG = "storage_littlefs";

esp_err_t storage_fat_init() {
    ESP_LOGI(TAG, "Mounting LittleFs filesystem");
    esp_vfs_littlefs_conf_t conf = {
        .base_path = base_path,
        .partition_label = "storage",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    return ESP_OK;
}

esp_err_t storage_fat_deinit() {
    ESP_LOGI(TAG, "Unmounting LittleFs filesystem");
    esp_vfs_littlefs_unregister("storage");
    return ESP_OK;
}

void get_free_space_kb(int* free, int* total) {
    size_t total_ = 0, used_ = 0;
    esp_littlefs_info("storage", &total_, &used_);
    *free = (total_ - used_) / 1024;
    *total = total_ / 1024;
}

void wdt_off() {
    wdt_hal_context_t wdt0ctx = {.inst = WDT_MWDT0, .mwdt_dev = &TIMERG0};
    wdt_hal_write_protect_disable(&wdt0ctx);
    wdt_hal_disable(&wdt0ctx);
    wdt_hal_write_protect_enable(&wdt0ctx);
    wdt_hal_context_t wdt1ctx = {.inst = WDT_MWDT1, .mwdt_dev = &TIMERG1};
    wdt_hal_write_protect_disable(&wdt1ctx);
    wdt_hal_disable(&wdt1ctx);
    wdt_hal_write_protect_enable(&wdt1ctx);
}