#include "fs.hpp"

extern "C" {
#include <esp_vfs.h>
#include <esp_vfs_fat.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/sdspi_host.h>
}

// internal memory
static const char *TAG = "storage_fat";

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

static esp_err_t storage_fat_internal_init(FsSettings *settings) {
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 4,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount(mount_point, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t storage_fat_internal_deinit() {
    ESP_LOGI(TAG, "Unmounting FAT filesystem");
    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount(mount_point, s_wl_handle));
    return ESP_OK;
}

// sd card
static sdmmc_card_t *card;
static sdmmc_host_t host;

static esp_err_t storage_fat_sdcard_init(FsSettings *settings) {
    int PIN_NUM_MOSI = settings->pin_mosi;
    int PIN_NUM_MISO = settings->pin_miso;
    int PIN_NUM_CLK = settings->pin_clk;
    int PIN_NUM_CS = settings->pin_cs;

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
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

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
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");
    spi_bus_free((spi_host_device_t)host.slot);
    return ESP_OK;
}

static bool is_internal_storage = true;
static bool is_init = false;

bool fs_init(FsSettings *settings) {
    if (is_init) return true;
    bool success{};
    if (settings->external_sd) {
        success = storage_fat_sdcard_init(settings) == ESP_OK;
    } else {
        success = storage_fat_internal_init(settings) == ESP_OK;
    }
    if (success) {
        is_init = true;
        is_internal_storage = !settings->external_sd;
    }
    return success;
}

void fs_deinit() {
    if (!is_init) return;
    if (!is_internal_storage) {
        storage_fat_sdcard_deinit();
    } else {
        storage_fat_internal_deinit();
    }
    is_init = false;
}

bool fs_free_space_kb(int *free, int *total) {
    FATFS *fs = 0;
    DWORD fre_clust, fre_sect, tot_sect;
    /* Get volume information and free clusters of drive 0 */
    FRESULT res = f_getfree("0:", &fre_clust, &fs);
    if (res != FR_OK) {
        if (free) *free = 0;
        if (total) *total = 0;
        return false;
    }
    unsigned long long kSectorBytes = fs->ssize;
    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;
    if (free) *free = kSectorBytes * fre_sect / 1024ULL;
    if (total) *total = kSectorBytes * tot_sect / 1024ULL;
    return true;
}