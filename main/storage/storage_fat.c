#include "storage_fat.h"

#include <esp_vfs.h>
#include <esp_vfs_fat.h>
#include <esp_system.h>
#include <esp_log.h>

static const char *TAG = "storage_fat";

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

const char *base_path = "/spiflash";

esp_err_t storage_fat_init()
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 4,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t storage_fat_deinit()
{
    ESP_LOGI(TAG, "Unmounting FAT filesystem");
    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle));
    return ESP_OK;
}