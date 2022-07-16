#pragma once

#include <initializer_list>
#include <cstring>
#include <memory>

extern "C" {
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_check.h>
#include <esp_err.h>
}

#include <nvs_handle.hpp>

struct Setting {
    const char* name;
    const char* desc;
    double min_value;
    double max_value;
    double default_value;
};

static const std::initializer_list<Setting> kSettingDescriptor = {
    {"disable_accel", "Disable accelerometer", 0.0, 1.0, 0.0},
    {"fixed_qp", "Quantization parameter", 8.0, 20.0, 14.0},
    {"pt_count", "PT?", 0.0, 3.0, 3.0},
    {"pt_cutoff", "PT? cutoff", 25.0, 500.0, 150.0},
    {"dyn_count", "Dyn filter count", 0.0, 8.0, 0.0},
    {"dyn_freq_min", "Dyn filter min freq", 100.0, 1000.0, 150.0},
    {"dyn_freq_max", "Dyn filter max freq", 100.0, 1000.0, 600.0},
    {"dyn_q", "Dyn filter Q", 1.0, 1000.0, 10.0},
    {"dyn_lr", "Dyn filter learning rate", 0.5, 10000.0, 300.0},
    {"dyn_lr_smooth", "Dyn filter tracking smoothing", 0.0001, 1.0, 0.01}};

class SettingsManager {
   public:
    SettingsManager() {
        esp_err_t err;
        nvs_handle = nvs::open_nvs_handle("settings", NVS_READWRITE, &err);
        ESP_ERROR_CHECK(err);
        for (auto& s : kSettingDescriptor) {
            double value;
            if (nvs_handle->get_blob(s.name, &value, 8) != ESP_OK) {
                Reset();
                break;
            }
        }
    }

    double Get(const char* name) {
        for (auto& s : kSettingDescriptor) {
            if (strcmp(s.name, name) == 0) {
                double value{};
                ESP_ERROR_CHECK(nvs_handle->get_blob(name, &value, 8));
                return value;
            }
        }
        return 0.0;
    }

    esp_err_t Set(const char* name, double value) {
        for (auto& s : kSettingDescriptor) {
            if (strcmp(s.name, name) == 0) {
                return nvs_handle->set_blob(name, &value, 8);
            }
        }
        return ESP_FAIL;
    }

    void Reset() {
        ESP_ERROR_CHECK(nvs_handle->erase_all());
        for (auto& s : kSettingDescriptor) {
            ESP_ERROR_CHECK(Set(s.name, s.default_value));
        }
    }

   private:
    std::shared_ptr<nvs::NVSHandle> nvs_handle;
};