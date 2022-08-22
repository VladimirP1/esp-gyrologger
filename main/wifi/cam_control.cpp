#include "cam_control.hpp"

#include "global_context.hpp"
#include "storage/settings.hpp"

extern "C" {
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"

#include <hal/gpio_types.h>
#include <driver/gpio.h>
}

#include <string>

#define TAG "camera_task"

void firefly_x_lite_task(void* param) {
    std::string rx_buffer;
    char host_ip[] = "192.168.42.1";
    int addr_family = 0;
    int ip_protocol = 0;

    rx_buffer.resize(128);

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(7878);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            close(sock);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        {
            int flags = fcntl(sock, F_GETFL);
            if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
                ESP_LOGE(TAG, "Unable to set socket non blocking");
            }
        }

        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, 7878);

        int err = connect(sock, (struct sockaddr*)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0 && errno != EINPROGRESS) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        {
            fd_set fdset;
            FD_ZERO(&fdset);
            FD_SET(sock, &fdset);
            struct timeval tv = {3, 0};
            int res = select(sock + 1, NULL, &fdset, NULL, &tv);
            if (res <= 0) {
                ESP_LOGE(TAG, "Socket unable to connect2: errno %d", errno);
                close(sock);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                continue;
            }
        }
        ESP_LOGI(TAG, "Successfully connected");

        {
            int flags = fcntl(sock, F_GETFL);
            if (fcntl(sock, F_SETFL, flags & ~O_NONBLOCK) == -1) {
                ESP_LOGE(TAG, "Unable to set socket blocking");
            }
        }

        const char session_start[] = R"_({"msg_id":257,"token":0})_";
        err = send(sock, session_start, strlen(session_start), 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }
        while (1) {
            int len = recv(sock, rx_buffer.data(), rx_buffer.size() - 1, 0);
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            } else {
                rx_buffer[len] = 0;
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer.c_str());
                if (rx_buffer.find("CAMERA_RECORD_START") != std::string::npos) {
                    gctx.logger_control.active = true;
                } else if (rx_buffer.find("CAMERA_RECORD_STOP") != std::string::npos) {
                    gctx.logger_control.active = false;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
}

void momentary_ground_task(void* param) {
    gpio_num_t trig_gpio = static_cast<gpio_num_t>(gctx.settings_manager->Get("trig_gpio_0"));

    if (trig_gpio < 0) {
        vTaskDelete(nullptr);
        return;
    }

    gpio_reset_pin(trig_gpio);
    gpio_set_direction(trig_gpio, GPIO_MODE_INPUT);

    bool prev_busy = false;
    while (1) {
        if (prev_busy != gctx.logger_control.busy) {
            prev_busy = gctx.logger_control.busy;
            gpio_set_direction(trig_gpio, GPIO_MODE_OUTPUT);
            gpio_set_level(trig_gpio, 0);
            vTaskDelay(300 / portTICK_PERIOD_MS);
            gpio_set_direction(trig_gpio, GPIO_MODE_INPUT);
            ESP_LOGI(TAG, "trigger!");
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void cam_control_task(void* param) {
    int type = gctx.settings_manager->Get("cam_ctrl_type");
    switch (type) {
        default:
        case 0: {
            vTaskDelete(nullptr);
        } break;
        case 1: {
            momentary_ground_task(param);
        } break;
        case 2: {
            firefly_x_lite_task(param);
        } break;
    }
}