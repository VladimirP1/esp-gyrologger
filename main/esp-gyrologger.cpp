// SPDX-License-Identifier: LGPL-2.1-or-later

extern "C" {
#include "bus/mini_i2c.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_console.h>
#include <nvs_flash.h>

#include <string.h>
#include <stdio.h>
}

#include "gyro/gyro.hpp"
#include "pipeline/gyro_ctx.hpp"
#include "pipeline/pt_filter.hpp"
#include "hal/fs.hpp"
#include "bus/aux_i2c.hpp"
#include "global_context.hpp"

#include "ebin-encoder-cpp/lib/writer.hpp"

#include <fcntl.h>
#include <unistd.h>

static const char* TAG = "main";

static void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

GyroHal gyro_hal{};
GyroCtx gyro_ctx{};

static uint8_t buf[2000];
static int8_t scratch[2000];
static quat::quat quats[5000];

static constexpr float kGyroToRads = 1.0 / 32.8 * 3.141592 / 180.0;
static constexpr float kAccelToG = 16.0 / 32767;

void app_main_cpp(void) {
    ESP_LOGI(TAG, "heap %u", esp_get_free_heap_size());

    nvs_init();
    gctx.aux_i2c_queue = xQueueCreate(1, sizeof(aux_i2c_msg_t));

    do {
        if (!gyro_hal_init(&gyro_hal, 5, 6)) {
            break;
        }
        if (!gyro_ctx_init(&gyro_ctx, &gyro_hal)) {
            break;
        }
        ESP_LOGI(TAG, "%s ready!", gyro_hal.gyro_type);
    } while (0);

    do {
        FsSettings fs_settings{
            .external_sd = true, .pin_mosi = 3, .pin_miso = 7, .pin_clk = 8, .pin_cs = 4};
        if (fs_init(&fs_settings)) {
            break;
        }
        fs_settings.external_sd = false;
        if (fs_init(&fs_settings)) {
            break;
        }
    } while (0);

    ESP_LOGI("main", "init done! free heap: %u", esp_get_free_heap_size());

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    int fd = open("/flash/test.bin", O_CREAT | O_TRUNC | O_WRONLY);

    size_t nwrite{};

    nwrite += writer::write_header(buf + nwrite, sizeof(buf) - nwrite);
    nwrite += writer::write_gyro_setup(gyro_ctx.gyr_block / gyro_ctx.gyr_div, buf + nwrite,
                                       sizeof(buf) - nwrite);
    nwrite += writer::write_accel_setup(4, buf + nwrite, sizeof(buf) - nwrite);
    if (write(fd, buf, nwrite) != nwrite) {
        ESP_LOGE("main", "write error");
    }
    nwrite = 0;
    fsync(fd);

    int iter{};
    quat::quat q{};
    quant::State state{};
    PtFilter gyro_filt{3, 125, gyro_hal.gyro_sr};
    PtFilter accel_filt{3, 15, gyro_hal.gyro_sr};
    while (1) {
        Descriptor desc{};
        while (!gyro_ctx.queue->pop(&desc)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        int16_t* gyro_ptr = (int16_t*)desc.ptr;
        int16_t* accel_ptr = (int16_t*)(desc.ptr + gyro_ctx.gyr_block * 6);

        // LPF
        for (size_t i = 0; i < gyro_ctx.gyr_block; ++i) {
            gyro_filt.apply3(gyro_ptr + 3 * i);
        }

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

        nwrite +=
            writer::write_gyro_data(state, quats, gyro_ctx.gyr_block / gyro_ctx.gyr_div,
                                    buf + nwrite, sizeof(buf) - nwrite, scratch, sizeof(scratch));
        nwrite += writer::write_accel_data(accel_ptr, gyro_ctx.acc_block / gyro_ctx.acc_div,
                                           buf + nwrite, sizeof(buf) - nwrite);
        nwrite += writer::write_time_block(desc.dt, buf + nwrite, sizeof(buf) - nwrite);

        if (write(fd, buf, nwrite) != nwrite) {
            ESP_LOGI("main", "write error");
        }
        ESP_LOGI("main", "write %u (%d kbytes/min)", nwrite,
                 nwrite * int(60.0 * gyro_hal.gyro_sr / gyro_ctx.gyr_block) / 1024);
        nwrite = 0;
        if (iter % 16 == 0) {
            ESP_LOGW("main", "fsync");
            fsync(fd);
        }
        ++iter;
        gyro_ctx.queue->free(&desc);
    }

    // const char s[] = "Hello, world!\n";
    // write(fd, s, strlen(s));
    // fsync(fd);

    // while (1) {
    //     for (int i = 0; i < 100; ++i) {
    //         Descriptor desc{};
    //         while (!gyro_ctx.queue->pop(&desc)) {
    //             vTaskDelay(10 / portTICK_PERIOD_MS);
    //         }
    //         gyro_ctx.queue->free(&desc);
    //         ESP_LOGI(TAG, "%d %d %d %u", desc.size1, desc.size2, desc.dt,
    //         esp_get_free_heap_size());
    //     }
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    // }
}

extern "C" {
void app_main(void) { app_main_cpp(); }
}