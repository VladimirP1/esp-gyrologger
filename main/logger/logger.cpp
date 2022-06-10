extern "C"
{
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

#define TAG "logger"

volatile bool logger_active = false;
volatile bool calibrate = false;

uint8_t buf[2000];
constexpr size_t kBlockSize = 1024;
constexpr int kScaleBits = 15;
constexpr double kQuantGyroScale = 3000;
constexpr double kQuantErrorLimit = 0.04;

static void logger_task_cpp(void *params_pvoid)
{
    gyro_task_params *params = (gyro_task_params *)params_pvoid;

    FILE *f = NULL;

    BasicIntegrator integrator;
    QuatEncoder quat_encoder;
    EntropyCoder entropy_coder(kScaleBits);

    uint16_t offset_gx{}, offset_gy{}, offset_gz{};
    uint16_t max_backlog = 0;
    TickType_t prev_dump = xTaskGetTickCount();
    bool file_cleared = false;
    for (int i = 0;; ++i)
    {
        gyro_sample_message msg;
        xQueueReceive(params->sample_queue, &msg, portMAX_DELAY);

        msg.gyro_x -= offset_gx;
        msg.gyro_y -= offset_gy;
        msg.gyro_z -= offset_gz;

        quat::quat q = integrator.update(msg);
        if (logger_active)
        {
            if (!f)
            {
                ESP_LOGI(TAG, "Opening file");
                f = fopen("/spiflash/gyro.bin", file_cleared ? "ab" : "wb");
                file_cleared = true;

                if (!f)
                {
                    ESP_LOGE("logger", "Cannot open file");
                }
            }
            if (i % 1000 == 0)
            {
                // auto r = quat::to_aa(q);
                ESP_LOGI("gyro_consumer", "i=%d, ts = %lld, qZ = %f, max_backlog = %d", i, msg.timestamp, q[3], max_backlog);
            }
            quat_encoder.encode(q);
        }
        else if (f)
        {
            fflush(f);
            fclose(f);
            f = NULL;
        }

        if (quat_encoder.has_block(kBlockSize))
        {
            auto block = quat_encoder.get_block(kBlockSize);
            uint8_t *buf_ptr = buf + sizeof(buf);
            const uint8_t *buf_end_ptr = buf + sizeof(buf);
            int out_buf_size = entropy_coder.encode_block(std::get<0>(block), std::get<1>(block), buf_ptr);
            quat_encoder.drop_block(kBlockSize);

            // auto pptr = buf_ptr;
            // decoder.decode_block(pptr, [](int x){printf("%d ", x);});
            while (buf_ptr != buf_end_ptr)
            {
                int written = fwrite(buf_ptr, 1, std::min(static_cast<size_t>(16), static_cast<size_t>(buf_end_ptr - buf_ptr)), f);
                buf_ptr += written;
                if (!written)
                {
                    // ESP_LOGI("gyro_consumer", "looping");
                    vTaskDelay(1);
                }
            }
            auto cur_dump = xTaskGetTickCount();
            double elapsed = (xTaskGetTickCount() - prev_dump) * 1.0 / configTICK_RATE_HZ;
            prev_dump = cur_dump;
            ESP_LOGI("gyro_consumer", "%d bytes, compression ratio = %.2f, logger capacity at this rate = %.2f h", (int)out_buf_size, out_buf_size * 1.0 / kBlockSize, 3000000 / (out_buf_size / elapsed) / 60 / 60);
        }

        max_backlog = msg.fifo_backlog > max_backlog ? msg.fifo_backlog : max_backlog;
        if (!logger_active)
            continue;
        if (calibrate)
        {
            offset_gx = msg.gyro_x;
            offset_gy = msg.gyro_y;
            offset_gz = msg.gyro_z;
            calibrate = false;
            ESP_LOGI("gyro_consumer", "gyro calibrated");
        }
        if (i % 1000 == 0)
        {
            // ESP_LOGI("gyro_consumer", "i=%d, ts = %lld, qZ = %f, max_backlog = %d", i, msg.timestamp, q[3], max_backlog);
            // ESP_LOGI("gyro_consumer", "i=%d, ts = %lld, gZ = %d, max_backlog = %d", i, msg.timestamp, msg.gyro_z, max_backlog);
            max_backlog = 0;
        }
        if (i % 2000 == 0)
        {
            fflush(f);
            fclose(f);
            f = NULL;
        }
    }
}

uint8_t buf2[2000];

static int do_logger_cmd(int argc, char **argv)
{
    if (argc == 2)
    {
        bool log = atoi(argv[1]);
        logger_active = log;

        ESP_LOGI("logger", "loger_enable = %d", log);
        return 0;
    }
    if (argc == 3)
    {
        if (strcmp(argv[1], "read") == 0)
        {
            // Open file for reading
            ESP_LOGI(TAG, "Reading file");
            FILE *f = fopen("/spiflash/gyro.bin", "rb");
            if (f == NULL)
            {
                ESP_LOGE(TAG, "Failed to open file for reading");
                return 1;
            }
            fseek(f, 0L, SEEK_END);
            ESP_LOGI("logger", "file size is %d bytes", ftell(f));
            fseek(f, 0L, SEEK_SET);

            EntropyDecoder decoder(kScaleBits, kBlockSize);

            int have_bytes = 0;
            while (true)
            {
                int read_bytes = fread(buf2 + have_bytes, 1, 2000 - have_bytes, f);
                have_bytes += read_bytes;

                if (have_bytes < 2000)
                    break;

                int bytes = 0;
                uint8_t *bptr = buf2;
                int decoded_bytes = decoder.decode_block(bptr, [&, m = 0](int x) mutable
                                                         { 
                                         if (++m%999==0) ESP_LOGI("logger", "decode: %d", x);
                                         bytes++; });
                have_bytes -= decoded_bytes;
                ESP_LOGI("logger", "%d block uncompressed to %d bytes", decoded_bytes, bytes);
                vTaskDelay(1);
                memmove(buf2, buf2 + decoded_bytes, have_bytes);
            }

            fclose(f);
            return 0;
        }
        if (strcmp(argv[1], "calibrate") == 0)
        {
            calibrate = true;
            return 0;
        }
    }
    return 1;
}

extern "C"
{
    void logger_task(void *params_pvoid)
    {
        logger_task_cpp(params_pvoid);
    }

    void register_logger_cmd()
    {
        const esp_console_cmd_t cmd = {
            .command = "logger",
            .help = NULL,
            .hint = NULL,
            .func = &do_logger_cmd,
            .argtable = NULL};
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
}