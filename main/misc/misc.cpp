#include "misc.hpp"

#include "global_context.hpp"
#include "storage/settings.hpp"
#include "wifi/wifi.hpp"

extern "C" {
#include <hal/gpio_types.h>
#include <driver/gpio.h>
#include <driver/sigmadelta.h>

#include <driver/rmt.h>
#include "led_strip/led_strip.h"

#include <freertos/task.h>
}

static gpio_num_t led_gpio{};
static gpio_num_t btn_gpio{};

void led_task(void* params) {
    led_gpio = static_cast<gpio_num_t>(gctx.settings_manager->Get("led_pin"));

    if (led_gpio < 0) {
        vTaskDelete(nullptr);
        return;
    }

    sigmadelta_config_t sigmadelta_cfg = {
        .channel = SIGMADELTA_CHANNEL_0,
        .sigmadelta_duty = -128,
        .sigmadelta_prescale = 80,
        .sigmadelta_gpio = static_cast<uint8_t>(led_gpio),
    };
    sigmadelta_config(&sigmadelta_cfg);
    gpio_set_drive_capability(led_gpio, GPIO_DRIVE_CAP_0);

    int duty = 0;
    bool dir = false;

    while (true) {
        if (gctx.logger_control.busy &&
            (esp_timer_get_time() - gctx.logger_control.last_block_time_us <= 2000000ULL) &&
            !gctx.logger_control.storage_failure) {
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

void led_strip_task(void* params) {
    led_gpio = static_cast<gpio_num_t>(gctx.settings_manager->Get("led_pin"));

    if (led_gpio < 0) {
        vTaskDelete(nullptr);
        return;
    }

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(led_gpio, RMT_CHANNEL_0);
    config.clk_div = 2;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(2, (led_strip_dev_t)config.channel);
    led_strip_t* strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE("led_strip_task", "install WS2812 driver failed");
        vTaskDelete(nullptr);
        return;
    }
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    uint32_t duty{};
    uint8_t k = gctx.settings_manager->Get("led_bright");

    while (true) {
        if (gctx.logger_control.busy &&
            (esp_timer_get_time() - gctx.logger_control.last_block_time_us <= 2000000ULL) &&
            !gctx.logger_control.storage_failure) {
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, k, 0, 0));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, 0, 0));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, k, 0, 0));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, 0, 0));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(800 / portTICK_PERIOD_MS);
            duty = 0;

        } else {
            // uint8_t v = (duty % 512 >= 256 ? 511 - duty : duty) * k / 100;
            // ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, v, 0));
            // ESP_ERROR_CHECK(strip->refresh(strip, 100));
            // duty = (duty + 4) % 512;

            // uint32_t r, g, b;
            // led_strip_hsv2rgb(duty, 100, k, &r, &g, &b);
            // ESP_ERROR_CHECK(strip->set_pixel(strip, 0, r, g, b));
            // duty = (duty + 1) % 360;
            // ESP_ERROR_CHECK(strip->refresh(strip, 100));
            // vTaskDelay(10 / portTICK_PERIOD_MS);

            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, duty >= 10 ? k : 0, 0));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            duty = (duty + 1) % 20;
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

void button_task(void* params) {
    if (gctx.settings_manager->Get("loop_mode") > 0.5) {
        gctx.logger_control.active = true;
    }

    btn_gpio = static_cast<gpio_num_t>(gctx.settings_manager->Get("btn_pin"));

    if (btn_gpio < 0) {
        vTaskDelete(nullptr);
        return;
    }

    gpio_set_direction(btn_gpio, GPIO_MODE_INPUT);
#if CONFIG_IDF_TARGET_ESP32C3
    if (btn_gpio != 19 && btn_gpio != 18) {
        gpio_set_pull_mode(btn_gpio, GPIO_PULLUP_ONLY);
    }
#elif CONFIG_IDF_TARGET_ESP32
    gpio_set_pull_mode(btn_gpio, GPIO_PULLUP_ONLY);
#endif

    static constexpr int kDebounceThreshold = 6;
    int debounce_counter = 0;

    bool cur_state = false;
    bool prev_state = false;

    while (true) {
        if (!gpio_get_level(btn_gpio)) {
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

            if (gctx.settings_manager->Get("wifi_stop_act") > 0.5) {
                if (gctx.logger_control.active) {
                    wifi_stop();
                } else {
                    wifi_start();
                }
            }
        }

        prev_state = cur_state;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}