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

#define EXAMPLE_ESP_WIFI_CHANNEL 11
#define EXAMPLE_MAX_STA_CONN 4

static const char *TAG = "wifi softAP";

class WifiManager {
   public:
    WifiManager(const char *ap_ssid, const char *ap_pass);

    static void EarlyInit();

   private:
    std::string ap_ssid_;
    std::string ap_pass_;
};

WifiManager::WifiManager(const char *ap_ssid, const char *ap_pass)
    : ap_ssid_(ap_ssid), ap_pass_(ap_pass) {}

void WifiManager::EarlyInit() {
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    }
}

static void wifi_init_sta(std::string ssid) {
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {.sta = {"Firefly_X_358AC2", "12345678"}};
    strcpy((char *)wifi_config.sta.ssid, ssid.c_str());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static std::string generate_ssid() {
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
    return "esplog_" + random_name(mac[3] << 16 | mac[4] << 8 | mac[5]);
}

static void wifi_init_softap() {
    std::string ssid = generate_ssid();
    wifi_config_t wifi_config = {.ap = {.ssid_len = (uint8_t)ssid.length(),
                                        .channel = EXAMPLE_ESP_WIFI_CHANNEL,
                                        .authmode = WIFI_AUTH_WPA_WPA2_PSK,
                                        .max_connection = EXAMPLE_MAX_STA_CONN}};
    strcpy((char *)wifi_config.ap.ssid, ssid.c_str());
    strcpy((char *)wifi_config.ap.password, "12345678");

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    if (gctx.settings_manager->Get("wifi_2dbm") > 0.5) {
        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(8));
    }
}

static std::string wifi_scan(void) {
    wifi_ap_record_t ap_info[15];
    uint16_t number = 15;
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_LOGI(TAG, "Total APs scanned = %u", number);
    for (int i = 0; (i < 15) && (i < number); i++) {
        ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
        ESP_LOGI(TAG, "Channel \t\t%d\n", ap_info[i].primary);
        if (strncmp((const char *)ap_info[i].ssid, "Firefly", 7) == 0) {
            ESP_ERROR_CHECK(esp_wifi_stop());
            return std::string{(const char *)ap_info[i].ssid};
        }
    }
    ESP_ERROR_CHECK(esp_wifi_stop());
    return "";
}

void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));

    auto ssid = wifi_scan();
    // wifi_init_sta(ssid);
    wifi_init_softap();
}

void wifi_start() { wifi_init_softap(); }

void wifi_stop() { esp_wifi_stop(); }