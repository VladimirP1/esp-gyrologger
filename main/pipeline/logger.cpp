#include "logger.hpp"

#include "ebin-encoder-cpp/lib/writer.hpp"
#include "pipeline/pt_filter.hpp"
#include "global_context.hpp"
#include "gyro/gyro.hpp"
#include "pipeline/gyro_ctx.hpp"
#include "storage/filenames.hpp"
#include "storage/settings.hpp"
#include "hal/fs.hpp"

#include <esp_log.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

static constexpr char TAG[] = "logger";

static esp_err_t find_good_filename(char *buf) {
    DIR *dp;
    struct dirent *ep;
    dp = opendir("/flash");
    int max_idx = 0;
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            std::string filename = ep->d_name;
            int idx = filename_to_index(filename);
            if (idx < 0) {
                unlink(("/flash/" + filename).c_str());
                continue;
            }
            max_idx = std::max(idx, max_idx);
        }
        (void)closedir(dp);
    } else {
        ESP_LOGE(TAG, "Couldn't open the directory");
        return ESP_FAIL;
    }
    static constexpr char templ[] = "/flash/L%c%c%05d.bin";
    int epoch = gctx.settings_manager->Get("file_epoch");
    if (epoch < max_idx / 100000) {
        epoch = max_idx / 100000;
    }

    if (epoch != max_idx / 100000) {
        ESP_LOGW(TAG, "%d != %d", epoch, max_idx / 100000);
        index_to_filename(100000 * epoch + 1, buf);
    } else {
        index_to_filename(100000 * epoch + (max_idx % 100000) + 1, buf);
    }
    return ESP_OK;
}

static esp_err_t delete_oldest() {
    DIR *dp;
    struct dirent *ep;
    dp = opendir("/flash");
    std::string file_to_delete{};
    int min_idx = std::numeric_limits<int>::max();
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            std::string filename = ep->d_name;
            if (!validate_file_name(filename.c_str())) {
                unlink(("/flash/" + filename).c_str());
                continue;
            }
            int idx = ((filename[1] - 'A') * 26 + (filename[2] - 'A')) * 100000 +
                      std::stoi(filename.substr(3, 5));
            if (idx < min_idx) {
                min_idx = idx;
                file_to_delete = "/flash/" + filename;
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

static uint8_t buf[2000];
static int8_t scratch[2000];
static quat::quat quats[5000];
static char file_name_buf[30];

static constexpr float kGyroToRads = 1.0 / 32.8 * 3.141592 / 180.0;
static constexpr float kAccelToG = 16.0 / 32767;

void logger_task(void *params_pvoid) {
    auto &gyro_hal = *gctx.gyro_hal;
    auto &gyro_ctx = *gctx.gyro_ctx;

    int fd = -1;

    quat::quat q{};
    quant::State state{};
    // todo: take these params from settings
    PtFilter gyro_filt{3, 150, gyro_hal.gyro_sr};
    PtFilter accel_filt{3, 5, gyro_hal.gyro_sr / gyro_hal.accel_div};

    TickType_t prev_dump = xTaskGetTickCount();
    size_t block_count{};
    size_t bytes_count{};
    while (true) {
        if (gctx.terminate_for_update) {
            ESP_LOGI(TAG, "Terminating for SW update");
            vTaskDelete(nullptr);
        }

        Descriptor desc{};
        while (!gyro_ctx.queue->pop(&desc)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        int16_t *gyro_ptr = (int16_t *)desc.ptr;
        int16_t *accel_ptr = (int16_t *)(desc.ptr + gyro_ctx.gyr_block * 6);

        xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY);
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

                // reinitialize encoder
                q = {};
                state = {};
            }

            if (fd < 0) {
                ESP_LOGI(TAG, "Opening file %s", gctx.logger_control.file_name);
                fd = open(gctx.logger_control.file_name, O_CREAT | O_TRUNC | O_WRONLY);
                if (fd < 0) {
                    ESP_LOGE(TAG, "Cannot open file: %s", strerror(errno));
                    gctx.logger_control.storage_failure = true;
                }

                // write header
                size_t nwrite{};
                nwrite += writer::write_header(buf + nwrite, sizeof(buf) - nwrite);
                nwrite += writer::write_gyro_setup(gyro_ctx.gyr_block / gyro_ctx.gyr_div,
                                                   buf + nwrite, sizeof(buf) - nwrite);
                nwrite += writer::write_accel_setup(gyro_ctx.acc_block / gyro_ctx.acc_div, 4,
                                                    buf + nwrite, sizeof(buf) - nwrite);
                nwrite += writer::write_imu_orient("xyz", buf + nwrite, sizeof(buf) - nwrite);
                if (write(fd, buf, nwrite) != nwrite) {
                    ESP_LOGE("main", "write error");
                }
                bytes_count = nwrite;
                fsync(fd);
            }

            // LPF
            // for (size_t i = 0; i < gyro_ctx.gyr_block; ++i) {
            //     gyro_filt.apply3(gyro_ptr + 3 * i);
            // }

            for (size_t i = 0; i < gyro_ctx.acc_block; ++i) {
                accel_filt.apply3(accel_ptr + 3 * i);
            }

            // integrate and decimate gyro
            float gscale = kGyroToRads * desc.dt / gyro_ctx.gyr_block * 1e-6;
            for (size_t i = 0; i < gyro_ctx.gyr_block; i++) {
                q = q * quat::quat{quat::vec{quat::base_type{gscale * gyro_ptr[3 * i + 0]},
                                             quat::base_type{gscale * gyro_ptr[3 * i + 1]},
                                             quat::base_type{gscale * gyro_ptr[3 * i + 2]}}};
                if (i % 10 == 0) q = q.normalized();
                if (i % gyro_ctx.gyr_div == 0) {
                    quats[i / gyro_ctx.gyr_div] = q;
                }
            }
            // auto qq = q.axis_angle();
            // ESP_LOGI("main", "q = %.2f %.2f %.2f", ((float)qq.x) * 180.0 / 3.14,
            //          ((float)qq.y) * 180.0 / 3.14, ((float)qq.z) * 180.0 / 3.14);

            // decimate accel
            for (size_t i = 0; i < gyro_ctx.acc_block / gyro_ctx.acc_div; ++i) {
                int new_index = 3 * i;
                int old_index = new_index * gyro_ctx.acc_div;
                accel_ptr[new_index] = accel_ptr[old_index];
            }

            size_t nwrite{};

            nwrite += writer::write_gyro_data(state, quats, gyro_ctx.gyr_block / gyro_ctx.gyr_div,
                                              buf + nwrite, sizeof(buf) - nwrite, scratch,
                                              sizeof(scratch));
            // nwrite += writer::write_accel_data(accel_ptr, gyro_ctx.acc_block /
            // gyro_ctx.acc_div,
            //                                    buf + nwrite, sizeof(buf) - nwrite);
            nwrite += writer::write_time_block(desc.dt, buf + nwrite, sizeof(buf) - nwrite);

            if (write(fd, buf, nwrite) != nwrite) {
                ESP_LOGI("main", "write error");
            }
            ESP_LOGI("main", "write %u (%d kbytes/min)", nwrite,
                     nwrite * int(60.0 * gyro_hal.gyro_sr / gyro_ctx.gyr_block) / 1024);
            gyro_ctx.queue->free(&desc);
            bytes_count += nwrite;
            ++block_count;

            auto cur_dump = xTaskGetTickCount();
        } else {
            if (fd >= 0) {
                fsync(fd);
                close(fd);
                fd = -1;
            }
            gctx.logger_control.busy = false;
            gctx.logger_control.file_name = NULL;
            xSemaphoreGive(gctx.logger_control.mutex);
            gyro_ctx.queue->free(&desc);
        }

        if (fd >= 0 && block_count % 20 == 0) {
            if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
                gctx.logger_control.total_bytes_written = bytes_count;
                int log_duration_ms =
                    xTaskGetTickCount() * portTICK_PERIOD_MS - gctx.logger_control.log_start_ts_ms;
                gctx.logger_control.avg_logging_rate_bytes_min =
                    gctx.logger_control.total_bytes_written * 1000LL * 60LL / log_duration_ms;

                xSemaphoreGive(gctx.logger_control.mutex);
            }
            fsync(fd);
        }
        int free{};
        fs_free_space_kb(&free, nullptr);
        if (free < 90 && free > 0) {
            delete_oldest();
        }
    }
}
// TickType_t prev_dump = xTaskGetTickCount();
// for (int i = 0;; ++i) {
//     if (gctx.terminate_for_update) {
//         ESP_LOGI(TAG, "Terminating for SW update");
//         vTaskDelete(nullptr);
//     }

//     GyroRing::WorkResult work_result = gctx.gyro_ring->Work();
//     if (!work_result.quats) {
//         vTaskDelay(1);
//         continue;
//     }

//     if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
//         gctx.logger_control.last_block_time_us = esp_timer_get_time();
//         if (gctx.logger_control.active) {
//             gctx.logger_control.busy = true;
//             gctx.logger_control.total_samples_written += kBlockSize;
//             xSemaphoreGive(gctx.logger_control.mutex);

//             if (!gctx.logger_control.file_name) {
//                 find_good_filename(file_name_buf);
//                 xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY);
//                 gctx.logger_control.file_name = file_name_buf;
//                 gctx.logger_control.total_samples_written = 0;
//                 gctx.logger_control.log_start_ts_ms = xTaskGetTickCount() *
//                 portTICK_PERIOD_MS; gctx.logger_control.avg_logging_rate_bytes_min = 0;
//                 xSemaphoreGive(gctx.logger_control.mutex);
//                 ESP_LOGI(TAG, "Using filename: %s", file_name_buf);
//                 encoder.reset();
//             }

//             if (!f) {
//                 ESP_LOGI(TAG, "Opening file %s", gctx.logger_control.file_name);
//                 f = fopen(gctx.logger_control.file_name, "ab");

//                 if (!f) {
//                     ESP_LOGE(TAG, "Cannot open file");
//                     gctx.logger_control.storage_failure = true;
//                 }
//             }

//             auto tmp = encoder.encode_block(work_result.quats);

//             auto &bytes_to_write = tmp.first;
//             auto &max_error_rad = tmp.second;

//             auto buf_ptr = bytes_to_write.begin();
//             const auto buf_end_ptr = bytes_to_write.end();
//             int out_buf_size = bytes_to_write.size();

//             while (buf_ptr != buf_end_ptr) {
//                 int written = fwrite(&(*buf_ptr), 1,
//                                      std::min(static_cast<size_t>(16),
//                                               static_cast<size_t>(buf_end_ptr - buf_ptr)),
//                                      f);
//                 buf_ptr += written;
//                 if (!written) {
//                     vTaskDelay(1);
//                 }
//             }
//             auto cur_dump = xTaskGetTickCount();
//             double elapsed = (xTaskGetTickCount() - prev_dump) * 1.0 / configTICK_RATE_HZ;
//             prev_dump = cur_dump;
//             ESP_LOGI(TAG, "%d bytes, capacity = %.2f h, max_error = % .4f ",
//             (int)out_buf_size,
//                      3000000 / (out_buf_size / elapsed) / 60 / 60,
//                      float(max_error_rad) * 180 / M_PI);

//             // Dump accelerometer data
//             if (work_result.accels_len) {
//                 uint8_t accel_count = work_result.accels_len / 3;
//                 fwrite(&accel_count, 1, 1, f);
//                 fwrite(work_result.accels, 2, work_result.accels_len, f);
//                 ESP_LOGI(TAG, "%d bytes of accelerometer data", accel_count * 6);
//             }

//         } else {
//             if (f) {
//                 fflush(f);
//                 fclose(f);
//                 f = NULL;
//             }
//             gctx.logger_control.busy = false;
//             gctx.logger_control.file_name = NULL;
//             xSemaphoreGive(gctx.logger_control.mutex);
//         }
//     }

//     static int flush_gate = 0;
//     if (f && (++flush_gate) % 20 == 0) {
//         if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
//             gctx.logger_control.total_bytes_written = ftell(f);
//             int log_duration_ms =
//                 xTaskGetTickCount() * portTICK_PERIOD_MS -
//                 gctx.logger_control.log_start_ts_ms;
//             gctx.logger_control.avg_logging_rate_bytes_min =
//                 gctx.logger_control.total_bytes_written * 1000LL * 60LL / log_duration_ms;
//             xSemaphoreGive(gctx.logger_control.mutex);
//         }
//         esp_vfs_fsync(fileno(f));
//     }
//     auto [free, total] = get_free_space_kb();
//     if (free < 90 && free > 0) {
//         delete_oldest();
//     }
// }
// }