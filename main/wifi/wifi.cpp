// SPDX-License-Identifier: LGPL-2.1-or-later

#include "wifi.hpp"
#include "global_context.hpp"
#include "storage/settings.hpp"

extern "C" {
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include <string.h>
}

#include "random_names.hpp"

#include <string>

static const char *TAG = "wifi";

static int timeout_task_epoch = 0;

static void wifi_timeout_task(void *params) {
    int cur_epoch = (int)params;
    const int tout = gctx.settings_manager->Get("wifi_timeout");

    if (int cnt = tout; cnt) {
        while (cnt--) {
            if (gctx.wifi_stations) {
                cnt = tout;
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "%d wifi seconds left", cnt);
            if (cur_epoch != timeout_task_epoch) {
                vTaskDelete(nullptr);
            }
        }
        wifi_stop();
    }
    vTaskDelete(nullptr);
}

void wifi_start_timeout() {
    xTaskCreate(wifi_timeout_task, "wifi-timeout-task", 4096, (void *)++timeout_task_epoch,
                configMAX_PRIORITIES - 8, nullptr);
}

static void retry_wifi_task(void *param) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_wifi_connect();
    vTaskDelete(nullptr);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        ++gctx.wifi_stations;
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        --gctx.wifi_stations;
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xTaskCreate(retry_wifi_task, "retry-wifi", 4096, NULL, configMAX_PRIORITIES - 8, NULL);
        ESP_LOGI(TAG, "retry to connect to the AP");
    }
}

static std::string generate_ssid() {
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
    return "esplog_" + random_name(mac[3] << 16 | mac[4] << 8 | mac[5]);
}

static void wifi_init_apsta() {
    auto sta_ssid = gctx.settings_manager->GetString("sta_ssid");
    auto sta_password = gctx.settings_manager->GetString("sta_pwd");
    auto ap_ssid = generate_ssid();
    auto ap_password = gctx.settings_manager->GetString("wifi_password");
    wifi_config_t wifi_config_sta = {};
    wifi_config_t wifi_config_ap = {.ap = {.ssid_len = (uint8_t)ap_ssid.length(),
                                           .channel = 11,
                                           .authmode = WIFI_AUTH_WPA_WPA2_PSK,
                                           .max_connection = 4}};
    strcpy((char *)wifi_config_ap.ap.ssid, ap_ssid.c_str());
    strcpy((char *)wifi_config_ap.ap.password, ap_password.c_str());
    strcpy((char *)wifi_config_sta.sta.ssid, sta_ssid.c_str());
    strcpy((char *)wifi_config_sta.sta.password, sta_password.c_str());
    ESP_ERROR_CHECK(esp_wifi_start());
    {
        static constexpr int wifi_dbm[] = {2, 5, 7, 8, 11, 13, 14, 15, 16, 18, 20};
        static constexpr int wifi_pwr[] = {8, 20, 28, 34, 44, 52, 56, 60, 66, 72, 80};
        double cfg_dbm = gctx.settings_manager->Get("wifi_dbm");
        int i = 10;
        while (i > 0 && wifi_dbm[i] > cfg_dbm) --i;
        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(wifi_pwr[i]));
    }
    if (gctx.sta_enabled) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta));
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));
    ESP_LOGI(TAG, "wifi_init_apsta finished.");
    gctx.wifi_active = true;
    wifi_start_timeout();
}

void wifi_init() {
    if (gctx.settings_manager->GetString("sta_ssid") != "-") {
        gctx.sta_enabled = true;
    }

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_init_apsta();
}

void wifi_start() {
    wifi_start_timeout();
    if (gctx.sta_enabled) {
        esp_wifi_set_mode(WIFI_MODE_APSTA);
    } else {
        wifi_init_apsta();
    }
    gctx.wifi_active = true;
}

void wifi_stop() {
    ++timeout_task_epoch;
    if (gctx.sta_enabled) {
        esp_wifi_set_mode(WIFI_MODE_STA);
    } else {
        esp_wifi_stop();
    }
    gctx.wifi_active = false;
}