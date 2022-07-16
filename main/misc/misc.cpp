#include "misc.hpp"

#include "global_context.hpp"
#include "wifi/wifi.hpp"

extern "C" {
#include <hal/gpio_types.h>
#include <driver/gpio.h>
#include <driver/sigmadelta.h>

#include <freertos/task.h>
}

#define LED_GPIO GPIO_NUM_5
#define BUTTON_GPIO GPIO_NUM_19

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
        if (gctx.logger_control.busy &&
            (esp_timer_get_time() - gctx.logger_control.last_block_time_us <= 2000000ULL)) {
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

void button_task(void* params) {
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);

    static constexpr int kDebounceThreshold = 6;
    int debounce_counter = 0;

    bool cur_state = false;
    bool prev_state = false;

    while (true) {
        if (!gpio_get_level(BUTTON_GPIO)) {
            if (debounce_counter == 1) {
                cur_state = true;
                debounce_counter = 0;
            } else if (debounce_counter > 0) {
                debounce_counter--;
            } else {
                debounce_counter = kDebounceThreshold;
            }
        } else {
            cur_state = false;
            debounce_counter = 0;
        }

        if (cur_state != prev_state && cur_state) {
            gctx.logger_control.active = !gctx.logger_control.active;

            vTaskDelay(200 / portTICK_PERIOD_MS);
            
            if (gctx.logger_control.active) {
                wifi_stop();
            } else {
                wifi_start();
            }
        }

        prev_state = cur_state;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}