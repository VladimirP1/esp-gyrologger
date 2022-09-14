#include "display.hpp"

extern "C" {
#include "u8g2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
}

#include "bus/aux_i2c.hpp"

#include "storage/utils.hpp"
#include "filters/gyro_ring.hpp"
#include "global_context.hpp"

#include <string>

#include "icons.hpp"

static uint8_t u8x8_gpio_and_delay_esplog(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                                          void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DELAY_NANO:  // delay arg_int * 1 nano second
            ets_delay_us(arg_int / 1000);
            break;
        case U8X8_MSG_DELAY_100NANO:  // delay arg_int * 100 nano seconds
            ets_delay_us(arg_int / 10);
            break;
        case U8X8_MSG_DELAY_10MICRO:  // delay arg_int * 10 micro seconds
            ets_delay_us(arg_int * 10);
            break;
        case U8X8_MSG_DELAY_MILLI:  // delay arg_int * 1 milli second
            vTaskDelay(arg_int / portTICK_PERIOD_MS);
            break;
        default:
            u8x8_SetGPIOResult(u8x8, 1);  // default return value
            break;
    }
    return 1;
}

static uint8_t u8x8_byte_esplog(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static aux_i2c_msg_t buf;
    uint8_t *data;

    switch (msg) {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0) {
                if (buf.len >= 32) break;
                buf.buf[buf.len++] = *data;
                data++;
                arg_int--;
            }
            break;
        case U8X8_MSG_BYTE_INIT:
            /* add your custom code to init i2c subsystem */
            break;
        case U8X8_MSG_BYTE_SET_DC:
            /* ignored for i2c */
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf.len = 0;
            buf.buf[buf.len++] = u8x8_GetI2CAddress(u8x8);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER: {
            if (buf.len) {
                aux_i2c_send_blocking(&buf);
                buf.len = 0;
            }
        } break;
        default:
            return 0;
    }
    return 1;
}

static u8g2_t u8g2;

void work_64x32() {
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawXBM(&u8g2, 0, 3, 64, 26, logo_64_26);
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    while (1) {
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
        u8g2_SetFontRefHeightText(&u8g2);
        u8g2_SetFontPosTop(&u8g2);

        if (gctx.wifi_active) {
            u8g2_DrawXBM(&u8g2, 54, 0, 7, 5, icon_wifi_7_5);
        } else {
            u8g2_DrawXBM(&u8g2, 54, 0, 7, 5, icon_nowifi_7_5);
        }
        u8g2_DrawStr(&u8g2, 61, 0, std::to_string(gctx.wifi_stations).c_str());

        int total_time_s{};
        if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
            std::string fname;
            if (gctx.logger_control.busy && gctx.logger_control.file_name) {
                total_time_s = (uint64_t)gctx.logger_control.total_samples_written *
                               (uint64_t)gctx.gyro_ring->GetInterval() / 1000000ULL;
                fname = std::string(gctx.logger_control.file_name).substr(10);
            } else {
                total_time_s = 0;
                fname = "-- IDLE --";
            }
            xSemaphoreGive(gctx.logger_control.mutex);
            u8g2_DrawStr(&u8g2, 0, 0, fname.c_str());
        }

        auto df_info = get_free_space_kb();
        char buf[32];
        static int byte_spinner_pos{};
        static int last_total_bytes_written{};
        char spinner[] = {'\\', '|', '/', '-'};
        if (gctx.logger_control.total_bytes_written != last_total_bytes_written) {
            last_total_bytes_written = gctx.logger_control.total_bytes_written;
            byte_spinner_pos = (byte_spinner_pos + 1) % 4;
        }
        snprintf(buf, 32, "SR %.1fk %02d:%02d %c%c",
                 gctx.logger_control.avg_sample_interval_ns != 0
                     ? 1e6 / gctx.logger_control.avg_sample_interval_ns
                     : .0,
                 total_time_s / 60, total_time_s % 60,
                 spinner[((uint64_t)(gctx.logger_control.last_block_time_us / 800e3)) % 4], spinner[byte_spinner_pos]);
        u8g2_DrawStr(&u8g2, 0, 6, buf);

        snprintf(buf, 32, "%.1f/%.1fM free", df_info.first / 1e3, df_info.second / 1e3);
        u8g2_DrawStr(&u8g2, 0, 12, buf);

        u8g2_SendBuffer(&u8g2);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void display_task(void *params) {
    int display_type = gctx.settings_manager->Get("display_type");
    switch (display_type) {
        case 0:
            vTaskDelete(nullptr);
            break;
        case 1:
            u8g2_Setup_ssd1306_i2c_64x32_1f_f(&u8g2, U8G2_R0, u8x8_byte_esplog,
                                              u8x8_gpio_and_delay_esplog);
            u8g2_SetI2CAddress(&u8g2, 0x78);
            u8g2_InitDisplay(&u8g2);
            u8g2_SetPowerSave(&u8g2, 0);
            work_64x32();
            break;
    }
}