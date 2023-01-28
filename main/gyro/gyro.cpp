#include "gyro.hpp"

#include "bus/aux_i2c.hpp"

extern "C" {
#include <esp_check.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "bus/mini_i2c.h"
}

void gyro_probe_and_start_task(GyroHal* hal);

bool gyro_hal_init(GyroHal* hal, int sda, int scl) {
    if (sda >= 0 && scl >= 0) {
        ESP_ERROR_CHECK(mini_i2c_init(sda, scl, 400000));
        gyro_probe_and_start_task(hal);
    } else {
        ESP_LOGW("gyro", "Please assign i2c gpio pins!");
        return false;
    }
    return true;
}
