#include "misc.hpp"

#include "global_context.hpp"

extern "C" {
#include <driver/gpio.h>
#include <driver/sigmadelta.h>

#include <freertos/task.h>
}

#define LED_GPIO GPIO_NUM_23

void led_task(void* params) {
    sigmadelta_config_t sigmadelta_cfg = {
        .channel = SIGMADELTA_CHANNEL_0,
        .sigmadelta_duty = 0,
        .sigmadelta_prescale = 80,
        .sigmadelta_gpio = LED_GPIO,
    };
    sigmadelta_config(&sigmadelta_cfg);
    gpio_set_drive_capability(LED_GPIO, GPIO_DRIVE_CAP_0);

    int duty = 0;
    bool dir = false;

    while (true) {
        if (gctx.logger_control.busy) {
            sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, 127);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, -128);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, 127);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, -128);
            vTaskDelay(800 / portTICK_PERIOD_MS);
            duty = 0;
        } else {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, duty - 128);
        duty_retry:
            duty += dir ? -4 : 4;
            if (duty > 255) {
                dir = true;
                goto duty_retry;
            } else if (duty < 0) {
                dir = false;
                goto duty_retry;
            }
        }
    }
}