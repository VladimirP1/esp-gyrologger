#include "cam_control.hpp"

#include "global_context.hpp"

#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"

#include <string>

#define TAG "camera_task"

void camera_task(void* param) {
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
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, 7878);

        int err = connect(sock, (struct sockaddr*)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");

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