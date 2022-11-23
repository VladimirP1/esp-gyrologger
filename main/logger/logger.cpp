// SPDX-License-Identifier: LGPL-2.1-or-later

#include "logger.hpp"
extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include <esp_log.h>
#include <esp_console.h>
#include <argtable3/argtable3.h>
#include "storage/storage_fat.h"
}

#include <cstring>

#include <fstream>

#include "compression/lib/compression.hpp"
#include "filters/gyro_ring.hpp"

#include "global_context.hpp"

#define TAG "logger"

bool validate_file_name(std::string &s) {
    if (s.size() != 12) return false;
    for (int i = 0; i < 3; ++i)
        if (char c = toupper(s[i]); c < 'A' || c > 'Z') return false;
    for (int i = 3; i < 8; ++i)
        if (char c = s[i]; c < '0' || c > '9') return false;
    if (char c = s[8]; c != '.') return false;
    for (int i = 9; i < 12; ++i)
        if (char c = toupper(s[i]); c < 'A' || c > 'Z') return false;
    return true;
}

static esp_err_t find_good_filename(char *buf) {
    DIR *dp;
    struct dirent *ep;
    dp = opendir("/spiflash");
    int max_idx = 0;
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            std::string filename = ep->d_name;
            if (!validate_file_name(filename)) {
                unlink(("/spiflash/" + filename).c_str());
                continue;
            }
            int idx = ((filename[1] - 'A') * 26 + (filename[2] - 'A')) * 100000 +
                      std::stoi(filename.substr(3, 5));
            max_idx = std::max(idx, max_idx);
        }
        (void)closedir(dp);
    } else {
        ESP_LOGE(TAG, "Couldn't open the directory");
        return ESP_FAIL;
    }
    static constexpr char templ[] = "/spiflash/L%c%c%05d.bin";
    int epoch = gctx.settings_manager->Get("file_epoch");
    if (epoch < max_idx / 100000) {
        epoch = max_idx / 100000;
    }

    if (epoch != max_idx / 100000) {
        ESP_LOGW(TAG, "%d != %d", epoch, max_idx / 100000);
        snprintf(buf, 30, templ, 'A' + epoch / 26, 'A' + epoch % 26, 1);
    } else {
        snprintf(buf, 30, templ, 'A' + epoch / 26, 'A' + epoch % 26, (max_idx % 100000) + 1);
    }
    return ESP_OK;
}

static esp_err_t delete_oldest() {
    DIR *dp;
    struct dirent *ep;
    dp = opendir("/spiflash");
    std::string file_to_delete{};
    int min_idx = std::numeric_limits<int>::max();
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            std::string filename = ep->d_name;
            if (!validate_file_name(filename)) {
                unlink(("/spiflash/" + filename).c_str());
                continue;
            }
            int idx = ((filename[1] - 'A') * 26 + (filename[2] - 'A')) * 100000 +
                      std::stoi(filename.substr(3, 5));
            if (idx < min_idx) {
                min_idx = idx;
                file_to_delete = "/spiflash/" + filename;
            }
        }
        (void)closedir(dp);
    } else {
        ESP_LOGE(TAG, "Couldn't open the directory");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Deleting %s", file_to_delete.c_str());
    unlink(file_to_delete.c_str());
    return ESP_OK;
}

extern "C" {
int esp_vfs_fsync(int fd);
}

static char file_name_buf[30];
void logger_task(void *params_pvoid) {
    FILE *f = NULL;

    // Coder encoder(kBlockSize, Coder::BitrateModeConstantQualityWithPressure(), .02 * M_PI /
    // 180.0);
    Coder encoder(kBlockSize, Coder::BitrateModeConstantQP(),
                  gctx.settings_manager->Get("fixed_qp"));

    TickType_t prev_dump = xTaskGetTickCount();
    for (int i = 0;; ++i) {
        if (gctx.terminate_for_update) {
            ESP_LOGI(TAG, "Terminating for SW update");
            vTaskDelete(nullptr);
        }

        GyroRing::WorkResult work_result = gctx.gyro_ring->Work();
        if (!work_result.quats) {
            vTaskDelay(1);
            continue;
        }

        if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
            gctx.logger_control.last_block_time_us = esp_timer_get_time();
            if (gctx.logger_control.active) {
                gctx.logger_control.busy = true;
                gctx.logger_control.total_samples_written += kBlockSize;
                xSemaphoreGive(gctx.logger_control.mutex);

                if (!gctx.logger_control.file_name) {
                    find_good_filename(file_name_buf);
                    xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY);
                    gctx.logger_control.file_name = file_name_buf;
                    gctx.logger_control.total_samples_written = 0;
                    gctx.logger_control.log_start_ts_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    gctx.logger_control.avg_logging_rate_bytes_min = 0;
                    xSemaphoreGive(gctx.logger_control.mutex);
                    ESP_LOGI(TAG, "Using filename: %s", file_name_buf);
                    encoder.reset();
                }

                if (!f) {
                    ESP_LOGI(TAG, "Opening file %s", gctx.logger_control.file_name);
                    f = fopen(gctx.logger_control.file_name, "ab");

                    if (!f) {
                        ESP_LOGE(TAG, "Cannot open file");
                        gctx.logger_control.storage_failure = true;
                    }
                }

                auto tmp = encoder.encode_block(work_result.quats);

                auto &bytes_to_write = tmp.first;
                auto &max_error_rad = tmp.second;

                auto buf_ptr = bytes_to_write.begin();
                const auto buf_end_ptr = bytes_to_write.end();
                int out_buf_size = bytes_to_write.size();

                while (buf_ptr != buf_end_ptr) {
                    int written = fwrite(&(*buf_ptr), 1,
                                         std::min(static_cast<size_t>(16),
                                                  static_cast<size_t>(buf_end_ptr - buf_ptr)),
                                         f);
                    buf_ptr += written;
                    if (!written) {
                        vTaskDelay(1);
                    }
                }
                auto cur_dump = xTaskGetTickCount();
                double elapsed = (xTaskGetTickCount() - prev_dump) * 1.0 / configTICK_RATE_HZ;
                prev_dump = cur_dump;
                ESP_LOGI(TAG, "%d bytes, capacity = %.2f h, max_error = % .4f ", (int)out_buf_size,
                         3000000 / (out_buf_size / elapsed) / 60 / 60,
                         float(max_error_rad) * 180 / M_PI);

                // Dump accelerometer data
                if (work_result.accels_len) {
                    uint8_t accel_count = work_result.accels_len / 3;
                    fwrite(&accel_count, 1, 1, f);
                    fwrite(work_result.accels, 2, work_result.accels_len, f);
                    ESP_LOGI(TAG, "%d bytes of accelerometer data", accel_count * 6);
                }

            } else {
                if (f) {
                    fflush(f);
                    fclose(f);
                    f = NULL;
                }
                gctx.logger_control.busy = false;
                gctx.logger_control.file_name = NULL;
                xSemaphoreGive(gctx.logger_control.mutex);
            }
        }

        static int flush_gate = 0;
        if (f && (++flush_gate) % 20 == 0) {
            if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
                gctx.logger_control.total_bytes_written = ftell(f);
                int log_duration_ms =
                    xTaskGetTickCount() * portTICK_PERIOD_MS - gctx.logger_control.log_start_ts_ms;
                gctx.logger_control.avg_logging_rate_bytes_min =
                    gctx.logger_control.total_bytes_written * 1000LL * 60LL / log_duration_ms;
                xSemaphoreGive(gctx.logger_control.mutex);
            }
            esp_vfs_fsync(fileno(f));
        }
        int free, total;
        get_free_space_kb(&free, &total);
        if (free < 90 && free > 0) {
            delete_oldest();
        }
    }
}