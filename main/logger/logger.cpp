// SPDX-License-Identifier: LGPL-2.1-or-later

#include "logger.hpp"
extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_console.h>
#include <argtable3/argtable3.h>
}

#include <cstring>

#include <fstream>

#include "compression/lib/compression.hpp"
#include "filters/gyro_ring.hpp"

#include "global_context.hpp"

#define TAG "logger"

static esp_err_t find_good_filename(char *buf) {
    static constexpr char templ[] = "/spiflash/log%03d.bin";
    for (int i = 100; i--;) {
        snprintf(buf, 30, templ, i);
        FILE *f = fopen(buf, "rb");
        if (f) {
            fclose(f);
            snprintf(buf, 30, templ, i + 1);
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

extern "C" {
int esp_vfs_fsync(int fd);
}

static char file_name_buf[30];
void logger_task(void *params_pvoid) {
    FILE *f = NULL;

    Coder encoder(kBlockSize, Coder::BitrateModeConstantQualityLimited(), .02 * M_PI / 180.0,
                  kBlockSize * 2);

    TickType_t prev_dump = xTaskGetTickCount();
    for (int i = 0;; ++i) {
        if (gctx.terminate_for_update) {
            ESP_LOGI(TAG, "Terminating for SW update");
            vTaskDelete(nullptr);
        }

        quat::quat *quat_block = gctx.gyro_ring->Work();
        if (!quat_block) {
            vTaskDelay(1);
            continue;
        }

        if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
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
                    ESP_LOGI(TAG, "Opening file");
                    f = fopen(gctx.logger_control.file_name, "ab");

                    if (!f) {
                        ESP_LOGE(TAG, "Cannot open file");
                    }
                }

                auto tmp = encoder.encode_block(quat_block);

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
        if (f && (++flush_gate) % 10 == 0) {
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
    }
}