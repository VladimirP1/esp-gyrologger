#include "gyro_mpu6050.h"
#include "gyro_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>

#include <driver/gptimer.h>

#include "bus/bus_i2c.h"

#include "global_context.h"

static const char *TAG = "gyro_mpu";

#define DEV_ADDR 0x68

#define REG_SMPRT_DIV 0x19

#define REG_CONFIG 0x1a
#define REG_CONFIG_BIT_DLPF_CFG_0 0
#define REG_CONFIG_VALUE_DLPF_188HZ 1


#define REG_GYRO_CONFIG 0x1b
#define REG_GYRO_CONFIG_BIT_FS_SEL_0 3
#define REG_GYRO_CONFIG_VALUE_FS_250_DPS 0
#define REG_GYRO_CONFIG_VALUE_FS_500_DPS 1
#define REG_GYRO_CONFIG_VALUE_FS_1000_DPS 2
#define REG_GYRO_CONFIG_VALUE_FS_2000_DPS 3

#define REG_ACCEL_CONFIG 0x1c
#define REG_ACCEL_CONFIG_BIT_FS_SEL_0 3
#define REG_ACCEL_CONFIG_VALUE_FS_2_G 0
#define REG_ACCEL_CONFIG_VALUE_FS_4_G 1
#define REG_ACCEL_CONFIG_VALUE_FS_8_G 2
#define REG_ACCEL_CONFIG_VALUE_FS_16_G 3

#define REG_FIFO_EN 0x23
#define REG_FIFO_EN_MASK_TEMP (1 << 7)
#define REG_FIFO_EN_MASK_GYRO ((1 << 6) | (1 << 5) | (1 << 4))
#define REG_FIFO_EN_MASK_ACCEL (1 << 3)

#define REG_INT_STATUS 0x3a
#define REG_INT_STATUS_MASK_FIFO_OFLOW (1 << 4)
#define REG_INT_STATUS_MASK_DATA_RDY (1 << 0)

#define REG_USER_CONTROL 0x6a
#define REG_USER_CONTROL_MASK_FIFO_EN (1 << 6)
#define REG_USER_CONTROL_MASK_FIFO_RESET (1 << 2)
#define REG_USER_CONTROL_MASK_SIG_COND_RESET (1 << 0)

#define REG_PWR_MGMT_1 0x6b
#define REG_PWR_MGMT_1_MASK_RESET (1 << 7)
#define REG_PWR_MGMT_1_MASK_SLEEP (1 << 6)
#define REG_PWR_MGMT_1_BIT_CLKSEL_0 0
#define REG_PWR_MGMT_1_VALUE_CLKSEL_ZGYRO 3

#define REG_FIFO_COUNT_H 0x72
#define REG_FIFO_COUNT_L 0x73
#define REG_FIFO_RW 0x74

#define REG_WHO_AM_I 0x75

#define FIFO_SAMPLE_SIZE 12

static bool IRAM_ATTR gyro_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    static uint8_t tmp_data[FIFO_SAMPLE_SIZE];
    static uint64_t samples_total = 0;

    BaseType_t high_task_awoken = pdFALSE;

    // i2c_register_read(DEV_ADDR, REG_INT_STATUS, tmp_data, 1);
    // if (tmp_data[0] & REG_INT_STATUS_MASK_FIFO_OFLOW)
    // {
    //     ESP_LOGI(TAG, "FIFO overflow!");
    //     while (true)
    //         ;
    // }

    i2c_register_read(DEV_ADDR, REG_FIFO_COUNT_H, tmp_data, 2);
    const int fifo_bytes = tmp_data[1] | (tmp_data[0] << 8);

    // if (tmp_data[0] & REG_INT_STATUS_MASK_DATA_RDY)
    if (fifo_bytes > 0)
    {
        i2c_register_read(DEV_ADDR, REG_FIFO_RW, tmp_data, FIFO_SAMPLE_SIZE);
        gyro_sample_message msg = {
            .timestamp = samples_total * 5000,
            .accel_x = (tmp_data[0] << 8) | tmp_data[1],
            .accel_y = (tmp_data[2] << 8) | tmp_data[3],
            .accel_z = (tmp_data[4] << 8) | tmp_data[5],
            .gyro_x = (tmp_data[6] << 8) | tmp_data[7],
            .gyro_y = (tmp_data[8] << 8) | tmp_data[9],
            .gyro_z = (tmp_data[10] << 8) | tmp_data[11],
            .fifo_backlog = fifo_bytes};
        ++samples_total;
        if (xQueueSendToBackFromISR(ctx.gyro_raw_queue, &msg, &high_task_awoken) == errQUEUE_FULL) {
            while(1);
        }
    }

    return high_task_awoken;
}

void gyro_mpu6050_task(void *params_pvoid)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_register_read(DEV_ADDR, REG_WHO_AM_I, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%X", data[0]);
    if (data[0] != 0x68)
    {
        ESP_LOGI(TAG, "Wrong WHO_AM_I value!");
        return;
    }

    /* Reset */
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1, REG_PWR_MGMT_1_MASK_RESET));
    ESP_LOGI(TAG, "IMU reset");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1, 0));
    ESP_LOGI(TAG, "IMU wake up");

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_PWR_MGMT_1, REG_PWR_MGMT_1_VALUE_CLKSEL_ZGYRO << REG_PWR_MGMT_1_BIT_CLKSEL_0));
    ESP_LOGI(TAG, "IMU change clock");

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_CONFIG, REG_CONFIG_VALUE_DLPF_188HZ << REG_CONFIG_BIT_DLPF_CFG_0));
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_SMPRT_DIV, 1));
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_GYRO_CONFIG, REG_GYRO_CONFIG_VALUE_FS_2000_DPS << REG_GYRO_CONFIG_BIT_FS_SEL_0));
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_ACCEL_CONFIG, REG_ACCEL_CONFIG_VALUE_FS_16_G << REG_ACCEL_CONFIG_BIT_FS_SEL_0));

    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_FIFO_EN, REG_FIFO_EN_MASK_ACCEL | REG_FIFO_EN_MASK_GYRO));
    ESP_ERROR_CHECK(i2c_register_write_byte(DEV_ADDR, REG_USER_CONTROL, REG_USER_CONTROL_MASK_FIFO_EN));

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gyro_timer_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 900,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = 1};

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    vTaskDelete(NULL);
}