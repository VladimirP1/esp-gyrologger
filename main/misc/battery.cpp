#include "battery.hpp"

#if EXPERIMENTAL_BATTERY
#include "global_context.hpp"
#include "storage/settings.hpp"

extern "C" {
#include <hal/gpio_types.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include <esp_sleep.h>

extern void esp_brownout_disable();
}

#define TAG "battery"

void battery_task(void *params) {
    gpio_num_t dcdc_gpio = static_cast<gpio_num_t>(gctx.settings_manager->Get("dcdc_en"));
    int volt_thresh = gctx.settings_manager->Get("volt_thresh");

    if (dcdc_gpio < 0) {
        vTaskDelete(nullptr);
        return;
    }

    gpio_reset_pin(dcdc_gpio);
    gpio_set_direction(dcdc_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dcdc_gpio, 1);

    int low_cnt = 0;

    esp_adc_cal_characteristics_t *adc_chars =
        (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, adc_chars);
    while (1) {
        uint32_t reading;
        esp_err_t ret = esp_adc_cal_get_voltage(ADC_CHANNEL_0, adc_chars, &reading);
        if (ret != ESP_OK) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        reading *= 2;
        ESP_LOGI(TAG, "%d %d", reading, low_cnt);

        gctx.battery_voltage_mv = reading;

        if (low_cnt > 10) {
            esp_brownout_disable();
            gpio_set_level(dcdc_gpio, 0);
            esp_deep_sleep_start();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (reading < volt_thresh) {
            ++low_cnt;
        } else {
            low_cnt = 0;
            gpio_set_level(dcdc_gpio, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif