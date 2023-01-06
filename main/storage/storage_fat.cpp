// SPDX-License-Identifier: LGPL-2.1-or-later

#include "storage_fat.hpp"
#include "settings.hpp"
#include "global_context.hpp"

extern "C" {
#include <esp_vfs.h>
#include <esp_vfs_fat.h>
#include <esp_system.h>
#include <esp_log.h>
#include <ff.h>
#include <driver/sdspi_host.h>
}

static const char *TAG = "storage_fat";

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

const char *base_path = "/spiflash";

static esp_err_t storage_fat_internal_init() {
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 4,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return ESP_FAIL;
    }
    return ESP_OK;
}

static sdmmc_card_t *card;
static sdmmc_host_t host;

static esp_err_t storage_fat_internal_deinit() {
    ESP_LOGI(TAG, "Unmounting FAT filesystem");
    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle));
    return ESP_OK;
}

static esp_err_t storage_fat_sdcard_init() {
    int PIN_NUM_MOSI = gctx.settings_manager->Get("sd_mosi");
    int PIN_NUM_MISO = gctx.settings_manager->Get("sd_miso");
    int PIN_NUM_CLK = gctx.settings_manager->Get("sd_sck");
    int PIN_NUM_CS = gctx.settings_manager->Get("sd_cs");

    if (PIN_NUM_MOSI < 0 || PIN_NUM_MISO < 0 || PIN_NUM_CLK < 0 || PIN_NUM_CS < 0) {
        return ESP_FAIL;
    }

    int ret{};

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true, .max_files = 5, .allocation_unit_size = 16 * 1024};

    ESP_LOGI(TAG, "Initializing SD card");
    {
        sdmmc_host_t _host = SDSPI_HOST_DEFAULT();
        host = _host;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ESP_FAIL;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)PIN_NUM_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(base_path, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. ");
        } else {
            ESP_LOGE(TAG,
                     "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    return ESP_OK;
}

static esp_err_t storage_fat_sdcard_deinit() {
    esp_vfs_fat_sdcard_unmount(base_path, card);
    ESP_LOGI(TAG, "Card unmounted");
    spi_bus_free((spi_host_device_t)host.slot);
    return ESP_OK;
}

static bool is_internal_storage = false;
esp_err_t storage_fat_init() {
    if (storage_fat_sdcard_init() != ESP_OK) {
        is_internal_storage = true;
        return storage_fat_internal_init();
    }
    return ESP_OK;
}

esp_err_t storage_fat_deinit() {
    if (is_internal_storage) {
        return storage_fat_internal_deinit();
    } else {
        return storage_fat_sdcard_deinit();
    }
}

std::pair<int, int> get_free_space_kb() {
    FATFS *fs = 0;
    DWORD fre_clust, fre_sect, tot_sect;
    /* Get volume information and free clusters of drive 0 */
    FRESULT res = f_getfree("0:", &fre_clust, &fs);
    if (res != FR_OK) {
        return {0, 0};
    }
    unsigned long long kSectorBytes = fs->ssize;
    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;
    return {kSectorBytes * fre_sect / 1024ULL, kSectorBytes * tot_sect / 1024ULL};
}