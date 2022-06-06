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

#include <rom/miniz.h>
}

#include <fstream>

#include "compression/compression.hpp"
#include "compression/integration.hpp"
#include "compression/quat.hpp"

#define TAG "logger"

volatile bool logger_active = false;
volatile bool calibrate = false;

static void logger_task_cpp(void *params_pvoid)
{
    gyro_task_params *params = (gyro_task_params *)params_pvoid;

    FILE *f;

    ESP_LOGI(TAG, "Opening file");
    f = fopen("/spiflash/hello.txt", "w");
    fprintf(f, "written using ESP-IDF %s\n", esp_get_idf_version());

    BasicIntegrator integrator;

    uint16_t offset_gx{}, offset_gy{}, offset_gz{};
    uint16_t max_backlog = 0;
    for (int i = 0;; ++i)
    {
        gyro_sample_message msg;
        xQueueReceive(params->sample_queue, &msg, portMAX_DELAY);
        
        msg.gyro_x -= offset_gx;
        msg.gyro_y -= offset_gy;
        msg.gyro_z -= offset_gz;

        quat::quat q = integrator.update(msg);

        max_backlog = msg.fifo_backlog > max_backlog ? msg.fifo_backlog : max_backlog;
        if (!logger_active)
            continue;
        if (calibrate) {
            offset_gx = msg.gyro_x;
            offset_gy = msg.gyro_y;
            offset_gz = msg.gyro_z;
            calibrate = false;
            ESP_LOGI("gyro_consumer", "gyro calibrated");
        }
        if (i % 100 == 0)
        {
            ESP_LOGI("gyro_consumer", "i=%d, ts = %lld, qZ = %f, max_backlog = %d", i, msg.timestamp, q[3], max_backlog);
            // ESP_LOGI("gyro_consumer", "i=%d, ts = %lld, gZ = %d, max_backlog = %d", i, msg.timestamp, msg.gyro_z, max_backlog);
            {
                fprintf(f, "i=%d, ts = %lld, gZ = %d, max_backlog = %d\n", i, msg.timestamp, msg.gyro_z, max_backlog);
            }
            max_backlog = 0;
        }
        if (i % 2000 == 0)
        {
            fclose(f);

            f = fopen("/spiflash/hello.txt", "a");
        }
    }
}

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
            FILE *f = fopen("/spiflash/hello.txt", "r");
            if (f == NULL)
            {
                ESP_LOGE(TAG, "Failed to open file for reading");
                return 1;
            }
            for (int i = 0; i < 100; ++i)
            {
                char line[128];
                if (!fgets(line, sizeof(line), f))
                    break;
                // strip newline
                char *pos = strchr(line, '\n');
                if (pos)
                {
                    *pos = '\0';
                }
                ESP_LOGI(TAG, "Read from file: '%s'", line);
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