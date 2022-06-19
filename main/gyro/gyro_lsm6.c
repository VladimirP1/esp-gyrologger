#include "gyro_lsm6.h"
#include "gyro_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_attr.h>
#include <driver/timer.h>

#include "bus/bus_i2c.h"

#include "global_context.h"

static const char* TAG = "gyro_lsm6";

#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL3_BIT_BDR_GY_0 4
#define REG_FIFO_CTRL3_BIT_BDR_XL_0 0

#define REG_FIFO_CTRL4 0x0A
#define REG_FIFO_CTRL4_BIT_FIFO_MODE_0 0

#define REG_WHO_AM_I 0x0F

#define REG_CTRL1_XL 0x10
#define REG_CTRL1_XL_BIT_ODR_XL_0 4
#define REG_CTRL1_XL_BIT_FS0_XL 2
#define REG_CTRL1_XL_BIT_LPF2_XL_EN 1

#define REG_CTRL2_G 0x11
#define REG_CTRL2_G_BIT_ODR_G0 4
#define REG_CTRL2_G_BIT_FS0_G 2

#define REG_CTRL3_C 0x12
#define REG_CTRL3_C_BIT_BDU 6
#define REG_CTRL3_C_BIT_IF_INC 2
#define REG_CTRL3_C_BIT_SW_RESET 0

#define REG_CTRL4_C 0x13
#define REG_CTRL4_C_BIT_LPF1_SEL_G 1

#define REG_CTRL6_C 0x15
#define REG_CTRL6_C_BIT_FTYPE_0 0

#define REG_FIFO_STATUS1 0x3a

#define REG_FIFO_STATUS2 0x3b
#define REG_FIFO_STATUS2_BIT_FIFO_OVR_IA 6
#define REG_FIFO_STATUS2_BIT_FIFO_FULL_IA 5
#define REG_FIFO_STATUS2_BIT_FIFO_OVR_LATCHED 3
#define REG_FIFO_STATUS2_BIT_DIFF_FIFO_8 0

#define REG_INTERNAL_FREQ_FINE 0x63

#define REG_FIFO_DATA_OUT_TAG 0x78
#define REG_FIFO_DATA_OUT_X_L 0x79
#define REG_FIFO_DATA_OUT_X_H 0x7A
#define REG_FIFO_DATA_OUT_Y_L 0x7B
#define REG_FIFO_DATA_OUT_Y_H 0x7C
#define REG_FIFO_DATA_OUT_Z_L 0x7D
#define REG_FIFO_DATA_OUT_Z_H 0x7E

#define REG_FIFO_DATA_OUT_TAG_VALUE_TAG_GYRO 1
#define REG_FIFO_DATA_OUT_TAG_VALUE_TAG_ACCEL 2

#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

static const uint8_t dev_adr = 0x6a;

static bool IRAM_ATTR gyro_timer_cb(void* args) {
    static uint8_t tmp_data[7];
    static uint64_t time = 0;
    static int16_t gyro[3];
    static int16_t accel[3];
    static bool have_gyro = false;

    if (gctx.pause_polling) {
        i2c_register_write_byte(dev_adr, REG_FIFO_CTRL4, (0 << REG_FIFO_CTRL4_BIT_FIFO_MODE_0));
        gctx.pause_polling = false;
        return false;
    } else if (gctx.continue_polling) {
        i2c_register_write_byte(dev_adr, REG_FIFO_CTRL4, (1 << REG_FIFO_CTRL4_BIT_FIFO_MODE_0));
        gctx.continue_polling = false;
        return false;
    }

    i2c_register_read(dev_adr, REG_FIFO_STATUS1, tmp_data, 2);
    const int fifo_bytes = tmp_data[0] | ((tmp_data[1] & 0x03) << 8);

    if (fifo_bytes > 0) {
        i2c_register_read(dev_adr, REG_FIFO_DATA_OUT_TAG, tmp_data, 1);
        i2c_register_read(dev_adr, REG_FIFO_DATA_OUT_X_L, tmp_data + 1, 6);
        uint8_t tag = (tmp_data[0] >> 3);
        if (tag == REG_FIFO_DATA_OUT_TAG_VALUE_TAG_GYRO) {
            gyro[0] = (int16_t)((tmp_data[2] << 8) | tmp_data[1]);
            gyro[1] = (int16_t)((tmp_data[4] << 8) | tmp_data[3]);
            gyro[2] = (int16_t)((tmp_data[6] << 8) | tmp_data[5]);
            have_gyro = true;
        } else if (tag == REG_FIFO_DATA_OUT_TAG_VALUE_TAG_ACCEL) {
            accel[0] = (int16_t)((tmp_data[0] << 8) | tmp_data[1]);
            accel[1] = (int16_t)((tmp_data[2] << 8) | tmp_data[3]);
            accel[2] = (int16_t)((tmp_data[4] << 8) | tmp_data[5]);
        }
    }

    if (fifo_bytes > 256) {
        while (1)
            ;
    }

    BaseType_t high_task_awoken = pdFALSE;
    if (have_gyro) {
        have_gyro = false;
        gyro_sample_message msg = {.timestamp = time,
                                   .accel_x = accel[0],
                                   .accel_y = accel[1],
                                   .accel_z = accel[2],
                                   .gyro_x = gyro[0],
                                   .gyro_y = gyro[1],
                                   .gyro_z = gyro[2],
                                   .fifo_backlog = fifo_bytes,
                                   .smpl_interval_ns = 0};
        if (xQueueSendToBackFromISR(gctx.gyro_raw_queue, &msg, &high_task_awoken) ==
            errQUEUE_FULL) {
            while (1)
                ;
        }
    }
    time += 500;
    return high_task_awoken;
}

void gyro_lsm6_task(void* params) {
    gctx.gyro_raw_to_rads =  (35e-3 * 3.141592 / 180.0);

    static uint8_t data[2];
    i2c_register_read(dev_adr, REG_WHO_AM_I, data, 1);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%X", data[0]);
    if (data[0] != 0x6b) {
        ESP_LOGI(TAG, "Wrong WHO_AM_I value!");
        return;
    }

    i2c_register_write_byte(dev_adr, REG_CTRL3_C, (3 << REG_CTRL3_C_BIT_SW_RESET));

    vTaskDelay(5);

    i2c_register_write_byte(dev_adr, REG_CTRL3_C,
                            (3 << REG_CTRL3_C_BIT_BDU) | (3 << REG_CTRL3_C_BIT_IF_INC));

    // clang-format off
    i2c_register_write_byte(dev_adr, REG_CTRL6_C, (3 << REG_CTRL6_C_BIT_FTYPE_0));
    i2c_register_write_byte(dev_adr, REG_CTRL4_C, (1 << REG_CTRL4_C_BIT_LPF1_SEL_G));
    i2c_register_write_byte(dev_adr, REG_CTRL2_G, (8 << REG_CTRL2_G_BIT_ODR_G0) | (2 << REG_CTRL2_G_BIT_FS0_G)); // 8 is 1.66 khz
    i2c_register_write_byte(dev_adr, REG_CTRL1_XL, (5 << REG_CTRL1_XL_BIT_ODR_XL_0) | (1 << REG_CTRL1_XL_BIT_FS0_XL)); // 5 is 208 hz
    i2c_register_write_byte(dev_adr, REG_FIFO_CTRL3, (8 << REG_FIFO_CTRL3_BIT_BDR_GY_0) | (5 << REG_FIFO_CTRL3_BIT_BDR_XL_0));
    i2c_register_write_byte(dev_adr, REG_FIFO_CTRL4, (1 << REG_FIFO_CTRL4_BIT_FIFO_MODE_0));
    // clang-format on

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 5e-4 * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, gyro_timer_cb, NULL, ESP_INTR_FLAG_IRAM);

    timer_start(TIMER_GROUP_0, TIMER_0);

    vTaskDelete(NULL);
}