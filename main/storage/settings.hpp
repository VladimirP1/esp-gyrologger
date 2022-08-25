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

struct StringSetting {
    const char* name;
    const char* desc;
    const char* default_value;
};

static const std::initializer_list<Setting> kSettingDescriptor = {
    {"sda_pin", "SDA pin", -1.0, 64.0, -1},
    {"scl_pin", "SCL pin", -1.0, 64.0, -1},
    {"led_pin", "Led pin", -1.0, 64.0, -1},
    {"led_type", "Led type(0 = normal, 1 = ws2812)", 0, 1, 0},
    {"led_bright", "Ws2812 led brightness (0-100)", 0, 100, 8},
    {"btn_pin", "Button pin", -1.0, 64.0, -1},
    {"wifi_2dbm", "Set TX power to 2dbm", 0.0, 1.0, 1.0},
    {"wifi_stop_act", "Disable wifi when started with button", 0.0, 1.0, 1.0},
    {"accel_ofs_x", "Accel offset X (g)", -1.0, 1.0, 0.0},
    {"accel_ofs_y", "Accel offset Y (g)", -1.0, 1.0, 0.0},
    {"accel_ofs_z", "Accel offset Z (g)", -1.0, 1.0, 0.0},
    {"fixed_qp", "Quantization parameter", 8.0, 20.0, 14.0},
    {"pt_count", "PT?", 0.0, 3.0, 3.0},
    {"pt_cutoff", "PT? cutoff", 25.0, 500.0, 150.0},
    {"acc_pt_count", "Accel PT?", 0.0, 3.0, 1.0},
    {"acc_pt_cutoff", "Accel PT? cutoff", 0.1, 500.0, 5.0},
    {"dyn_count", "Dyn filter count", 0.0, 8.0, 0.0},
    {"dyn_freq_min", "Dyn filter min freq", 100.0, 1000.0, 200.0},
    {"dyn_freq_max", "Dyn filter max freq", 100.0, 1000.0, 600.0},
    {"dyn_q", "Dyn filter Q", 1.0, 1000.0, 20.0},
    {"dyn_lr", "Dyn filter learning rate", 0.5, 10000.0, 300.0},
    {"dyn_lr_smooth", "Dyn filter tracking smoothing", 0.0001, 1.0, 0.005},
    {"cam_ctrl_type", "Cam control type", 0, 2, 0},
    {"trig_gpio_0", "Momentary ground pin", -1, 64.0, -1},
    {"loop_mode", "Start logging on boot", 0, 1, 0},
};

static const std::initializer_list<StringSetting> kStringSettingDescriptor = {
    {"imu_orientation", "IMU Orientation", "xyz"},
    {"wifi_password", "WiFi password", "12345678"},
    {"sta_ssid", "WiFi station ssid", "-"},
    {"sta_pwd", "WiFi station password", "12345678"},
};

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
        for (auto& s : kStringSettingDescriptor) {
            char tmp[128];
            if (nvs_handle->get_string(s.name, tmp, 128) != ESP_OK) {
                Reset();
                break;
            }
        }
    }

    bool IsStringValue(const char* name) {
        for (auto& s : kStringSettingDescriptor) {
            if (strcmp(s.name, name) == 0) {
                return true;
            }
        }
        return false;
    }

    bool IsDoubleValue(const char* name) {
        for (auto& s : kSettingDescriptor) {
            if (strcmp(s.name, name) == 0) {
                return true;
            }
        }
        return false;
    }

    double Get(const char* name) {
        if (IsDoubleValue(name)) {
            double value{};
            ESP_ERROR_CHECK(nvs_handle->get_blob(name, &value, 8));
            return value;
        }
        return 0.0;
    }

    std::string GetString(const char* name) {
        if (IsStringValue(name)) {
            std::string s;
            s.resize(128);
            ESP_ERROR_CHECK(nvs_handle->get_string(name, s.data(), s.size()));
            s.resize(strlen(s.data()));
            return s;
        }

        return {};
    }

    esp_err_t Set(const char* name, double value) {
        if (IsDoubleValue(name)) {
            return nvs_handle->set_blob(name, &value, 8);
        }
        return ESP_FAIL;
    }

    esp_err_t SetString(const char* name, const char* value) {
        if (IsStringValue(name)) {
            return nvs_handle->set_string(name, value);
        }
        return ESP_FAIL;
    }

    void Reset() {
        ESP_ERROR_CHECK(nvs_handle->erase_all());
        for (auto& s : kSettingDescriptor) {
            ESP_ERROR_CHECK(Set(s.name, s.default_value));
        }
        for (auto& s : kStringSettingDescriptor) {
            ESP_ERROR_CHECK(SetString(s.name, s.default_value));
        }
    }

   private:
    std::shared_ptr<nvs::NVSHandle> nvs_handle;
};
