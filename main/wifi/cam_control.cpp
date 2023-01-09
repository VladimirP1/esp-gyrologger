#include "cam_control.hpp"

#include "global_context.hpp"
#include "storage/settings.hpp"

extern "C" {
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"

#include <hal/gpio_types.h>
#include <driver/gpio.h>
#include <driver/uart.h>
}

#include <string>

#define TAG "camera_task"

static bool make_tcp_conn(char* host, int port, int* sock_ret) {
    int addr_family = 0;
    int ip_protocol = 0;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(host);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        close(sock);
        return false;
    }

    {
        int flags = fcntl(sock, F_GETFL);
        if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
            ESP_LOGE(TAG, "Unable to set socket non blocking");
        }
    }

    ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host, port);

    int err = connect(sock, (struct sockaddr*)&dest_addr, sizeof(struct sockaddr_in6));
    if (err != 0 && errno != EINPROGRESS) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        close(sock);
        return false;
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
            return false;
        }
    }
    ESP_LOGI(TAG, "Successfully connected");

    {
        int flags = fcntl(sock, F_GETFL);
        if (fcntl(sock, F_SETFL, flags & ~O_NONBLOCK) == -1) {
            ESP_LOGE(TAG, "Unable to set socket blocking");
            close(sock);
            return false;
        }
    }
    *sock_ret = sock;
    return true;
}

void firefly_x_lite_task(void* param) {
    std::string rx_buffer;
    rx_buffer.resize(128);
    int sock{}, err{};
    while (1) {
        if (!make_tcp_conn("192.168.42.1", 7878, &sock)) {
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
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
                ESP_LOGI(TAG, "Received %d bytes", len);
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

void gopro_wifi_task(void* param) {
    bool prev_active = false;

    int err{}, sock{};
    while (1) {
        if (prev_active != gctx.logger_control.active) {
            prev_active = gctx.logger_control.active;
            if (!make_tcp_conn("10.5.5.9", 80, &sock)) {
                continue;
            }
            auto record_start = "GET /gp/gpControl/command/shutter?p=1 HTTP/1.1\n\n";
            auto record_stop = "GET /gp/gpControl/command/shutter?p=0 HTTP/1.1\n\n";
            if (gctx.logger_control.active) {
                err = send(sock, record_start, strlen(record_start), 0);
            } else {
                err = send(sock, record_stop, strlen(record_stop), 0);
            }
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            }
            // vTaskDelay(2000 / portTICK_PERIOD_MS);
            shutdown(sock, 0);
            close(sock);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void momentary_ground_task(void* param) {
    gpio_num_t trig_gpio = static_cast<gpio_num_t>(gctx.settings_manager->Get("trig_gpio_0"));

    if (trig_gpio < 0) {
        vTaskDelete(nullptr);
        return;
    }

    bool invert = (bool) param;
    
    gpio_reset_pin(trig_gpio);
    gpio_set_direction(trig_gpio, GPIO_MODE_INPUT);

    bool prev_busy = false;
    while (1) {
        if (prev_busy != gctx.logger_control.busy) {
            prev_busy = gctx.logger_control.busy;
            gpio_set_direction(trig_gpio, GPIO_MODE_OUTPUT);
            gpio_set_level(trig_gpio, invert);
            vTaskDelay(300 / portTICK_PERIOD_MS);
            gpio_set_direction(trig_gpio, GPIO_MODE_INPUT);
            ESP_LOGI(TAG, "trigger!");
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void runcam_protocol_listen_task(void* param) {
    gpio_num_t rx_gpio = static_cast<gpio_num_t>(gctx.settings_manager->Get("trig_gpio_0"));

    if (rx_gpio < 0) {
        vTaskDelete(nullptr);
        return;
    }

    gpio_reset_pin(rx_gpio);

    uart_config_t uart_config = {
        .baud_rate = static_cast<int>(gctx.settings_manager->Get("rc_uart_baud")),
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    static constexpr int kBufSize = 256;

    ESP_ERROR_CHECK(uart_driver_install(0, kBufSize * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(0, &uart_config));
    ESP_ERROR_CHECK(
        uart_set_pin(0, UART_PIN_NO_CHANGE, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uint8_t buf[4];
    int pos = 0;
    while (1) {
        uart_read_bytes(0, buf + pos, 1, portMAX_DELAY);
        if (buf[(pos - 3 + 4) % 4] == 0xcc && buf[(pos - 2 + 4) % 4] == 0x01) {
            if (buf[(pos - 1 + 4) % 4] == 0x01 && buf[pos] == 0xe7) {
                ESP_LOGI(TAG, "runcam protocol: simulate power btn");
                gctx.logger_control.active = !gctx.logger_control.active;
            } else if (buf[(pos - 1 + 4) % 4] == 0x03 && buf[pos] == 0x98) {
                ESP_LOGI(TAG, "runcam protocol: record start");
                gctx.logger_control.active = true;
            } else if (buf[(pos - 1 + 4) % 4] == 0x04 && buf[pos] == 0xcc) {
                ESP_LOGI(TAG, "runcam protocol: record end");
                gctx.logger_control.active = false;
            }
        }
        pos = (pos + 1) % 4;
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
            momentary_ground_task((void*)false);
        } break;
        case 2: {
            firefly_x_lite_task(param);
        } break;
        case 3: {
            runcam_protocol_listen_task(param);
        } break;
        case 4: {
            gopro_wifi_task(param);
        } break;
        case 5: {
            momentary_ground_task((void*)true);
        } break;
    }
}