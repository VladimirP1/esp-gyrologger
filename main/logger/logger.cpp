extern "C" {
#include "logger.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_console.h>
#include <argtable3/argtable3.h>

#include "gyro/gyro_types.h"

#include <string.h>
}

#include <fstream>

#include "compression/compression.hpp"
#include "compression/integration.hpp"
#include "compression/quat.hpp"

#include "global_context.h"

#define TAG "logger"

uint8_t buf[2000];
constexpr size_t kBlockSize = 1024;
constexpr int kScaleBits = 15;
constexpr double kQuantGyroScale = 3000;
constexpr double kQuantErrorLimit = 0.04;

static void logger_task_cpp(void *params_pvoid) {
    FILE *f = NULL;

    BasicIntegrator integrator;
    QuatEncoder quat_encoder;
    EntropyCoder entropy_coder(kScaleBits);

    uint16_t offset_gx{}, offset_gy{}, offset_gz{};
    TickType_t prev_dump = xTaskGetTickCount();
    bool file_cleared = false;
    for (int i = 0;; ++i) {
        gyro_sample_message msg;
        xQueueReceive(ctx.gyro_raw_queue, &msg, portMAX_DELAY);

        msg.gyro_x -= offset_gx;
        msg.gyro_y -= offset_gy;
        msg.gyro_z -= offset_gz;

        quat::quat q = integrator.update(msg);
        if (xSemaphoreTake(ctx.logger_control.mutex, 0)) {
            if (ctx.logger_control.active) {
                ctx.logger_control.busy = true;
                if (ctx.logger_control.calibration_pending) {
                    offset_gx = msg.gyro_x;
                    offset_gy = msg.gyro_y;
                    offset_gz = msg.gyro_z;
                    ctx.logger_control.calibration_pending = false;
                }
                xSemaphoreGive(ctx.logger_control.mutex);

                if (!f) {
                    ESP_LOGI(TAG, "Opening file");
                    f = fopen("/spiflash/gyro.bin", file_cleared ? "ab" : "wb");
                    file_cleared = true;

                    if (!f) {
                        ESP_LOGE("logger", "Cannot open file");
                    }
                }
                if (i % 1000 == 0) {
                    ESP_LOGI("gyro_consumer", "i=%d, ts = %lld, qZ = %f", i, msg.timestamp, q[3]);
                }
                quat_encoder.encode(q);

                if (quat_encoder.has_block(kBlockSize)) {
                    auto block = quat_encoder.get_block(kBlockSize);
                    uint8_t *buf_ptr = buf + sizeof(buf);
                    const uint8_t *buf_end_ptr = buf + sizeof(buf);
                    int out_buf_size =
                        entropy_coder.encode_block(std::get<0>(block), std::get<1>(block), buf_ptr);
                    quat_encoder.drop_block(kBlockSize);

                    while (buf_ptr != buf_end_ptr) {
                        int written = fwrite(buf_ptr, 1,
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
                    ESP_LOGI(
                        "gyro_consumer",
                        "%d bytes, compression ratio = %.2f, logger capacity at this rate = %.2f h",
                        (int)out_buf_size, out_buf_size * 1.0 / kBlockSize,
                        3000000 / (out_buf_size / elapsed) / 60 / 60);
                }
            } else {
                xSemaphoreGive(ctx.logger_control.mutex);
            }
        } else if (f) {
            fflush(f);
            fclose(f);
            f = NULL;
            ctx.logger_control.busy = false;
        }

        if (f && (i % 2000 == 0)) {
            fflush(f);
            fclose(f);
            f = NULL;
        }
    }
}

uint8_t buf2[2000];

static int do_logger_cmd(int argc, char **argv) {
    if (argc == 2) {
        bool log = atoi(argv[1]);
        ctx.logger_control.active = log;

        ESP_LOGI("logger", "loger_enable = %d", log);
        return 0;
    }
    if (argc == 3) {
        if (strcmp(argv[1], "read") == 0) {
            // Open file for reading
            ESP_LOGI(TAG, "Reading file");
            FILE *f = fopen("/spiflash/gyro.bin", "rb");
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file for reading");
                return 1;
            }
            fseek(f, 0L, SEEK_END);
            ESP_LOGI("logger", "file size is %d bytes", ftell(f));
            fseek(f, 0L, SEEK_SET);

            EntropyDecoder decoder(kScaleBits, kBlockSize);
            QuatDecoder quat_decoder;

            int have_bytes = 0;
            while (true) {
                int read_bytes = fread(buf2 + have_bytes, 1, 2000 - have_bytes, f);
                have_bytes += read_bytes;

                if (have_bytes < 2000) break;

                int bytes = 0;
                int decoded_bytes = decoder.decode_block(buf2, [&, m = 0](int x) mutable {
                    if (++m % 999 == 0) ESP_LOGI("logger", "decode: %d", x);
                    quat_decoder.decode(&x, &x + 1, [](const quat::quat &q) {
                        // printf("q {%.3f, %.3f, %.3f, %.3f}\n", q[0], q[1], q[2], q[3]);
                    });
                    bytes++;
                });
                have_bytes -= decoded_bytes;
                ESP_LOGI("logger", "%d block uncompressed to %d bytes", decoded_bytes, bytes);

                vTaskDelay(1);
                memmove(buf2, buf2 + decoded_bytes, have_bytes);
            }

            fclose(f);
            return 0;
        }
        if (strcmp(argv[1], "calibrate") == 0) {
            ctx.logger_control.calibration_pending = true;
            return 0;
        }
    }
    return 1;
}

extern "C" {
void logger_task(void *params_pvoid) { logger_task_cpp(params_pvoid); }

void register_logger_cmd() {
    const esp_console_cmd_t cmd = {
        .command = "logger", .help = NULL, .hint = NULL, .func = &do_logger_cmd, .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
}