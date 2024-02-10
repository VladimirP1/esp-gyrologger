#include "display.hpp"

extern "C" {
#include "u8g2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
}

#include "bus/aux_i2c.hpp"

#include "filters/gyro_ring.hpp"
#include "global_context.hpp"
#include "storage/storage_fat.hpp"

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

static bool display_on{};
static u8g2_t u8g2;
static SemaphoreHandle_t display_mtx;

void work_64x32() {
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawXBM(&u8g2, 0, 3, 64, 26, logo_64_26);
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    auto check_log_start = []() {
        static bool prev_busy{};
        if (prev_busy != gctx.logger_control.busy) {
            u8g2_SendF(&u8g2, "c", 0xa7);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            u8g2_SendF(&u8g2, "c", 0xa6);
            prev_busy = gctx.logger_control.busy;
        }
    };

    auto redraw = []() {
        xSemaphoreTake(display_mtx, portMAX_DELAY);
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
        u8g2_SetFontRefHeightText(&u8g2);
        u8g2_SetFontPosTop(&u8g2);

        static int total_time_s{};
        static std::string fname{"-IDLE--"};
        if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
            if (gctx.logger_control.busy && gctx.logger_control.file_name) {
                total_time_s = (uint64_t)gctx.logger_control.total_samples_written *
                               (uint64_t)gctx.gyro_ring->GetInterval() / 1000000ULL;
                fname = std::string(gctx.logger_control.file_name).substr(10);
                fname = fname.substr(1, 7);
            }
            xSemaphoreGive(gctx.logger_control.mutex);
        }

        u8g2_SetFont(&u8g2, u8g2_font_9x15_tf);
        u8g2_SetFontRefHeightText(&u8g2);
        u8g2_SetFontPosTop(&u8g2);
        u8g2_DrawStr(&u8g2, 0, 0, fname.c_str());

        auto df_info = get_free_space_kb();
        char buf[32];
        if (df_info.first < 10000) {
            snprintf(buf, 32, "%02d:%02d %.1fM%c", total_time_s / 60, total_time_s % 60,
                     df_info.first / 1e3, gctx.wifi_active ? '!' : ' ');
        } else {
            snprintf(buf, 32, "%02d:%02d %.1fG%c", total_time_s / 60, total_time_s % 60,
                     df_info.first / 1e6, gctx.wifi_active ? '!' : ' ');
        }

        u8g2_SetFont(&u8g2, u8g2_font_mozart_nbp_tr);
        u8g2_SetFontRefHeightText(&u8g2);
        u8g2_SetFontPosTop(&u8g2);
        u8g2_DrawStr(&u8g2, 0, 13, buf);
        xSemaphoreGive(display_mtx);
    };

    auto redraw_gyro = []() {
        xSemaphoreTake(gctx.logger_control.accel_raw_mtx, portMAX_DELAY);
        int gx = gctx.logger_control.gyro_raw[0] * 10;
        int gy = gctx.logger_control.gyro_raw[1] * 10;
        int gz = gctx.logger_control.gyro_raw[2] * 10;

        int ax = gctx.logger_control.accel_raw[0] * 8;
        int ay = gctx.logger_control.accel_raw[1] * 8;
        int az = gctx.logger_control.accel_raw[2] * 8;
        xSemaphoreGive(gctx.logger_control.accel_raw_mtx);

        xSemaphoreTake(display_mtx, portMAX_DELAY);
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawBox(&u8g2, 0, 24, 64, 8);
        u8g2_SetDrawColor(&u8g2, 1);

        if (gctx.logger_control.busy) {
            static int pos = 0;
            for (int i = 0; i <= 64; i += 16) {
                u8g2_DrawBox(&u8g2, (pos + i) % (64 + 16) - 8, 24, 8, 3);
            }
            pos += 4;
        }

        u8g2_DrawLine(&u8g2, 10, 28, std::min(std::max(10 + gx, 0), 20), 28);
        u8g2_DrawLine(&u8g2, 10, 28, 10, 31);

        u8g2_DrawLine(&u8g2, 30, 28, std::min(std::max(30 + gy, 20), 40), 28);
        u8g2_DrawLine(&u8g2, 30, 28, 30, 31);

        u8g2_DrawLine(&u8g2, 50, 28, std::min(std::max(50 + gz, 40), 60), 28);
        u8g2_DrawLine(&u8g2, 50, 28, 50, 31);

        u8g2_DrawLine(&u8g2, 10, 30, std::min(std::max(10 + ax, 0), 20), 30);
        u8g2_DrawLine(&u8g2, 10, 30, 10, 31);

        u8g2_DrawLine(&u8g2, 30, 30, std::min(std::max(30 + ay, 20), 40), 30);
        u8g2_DrawLine(&u8g2, 30, 30, 30, 31);

        u8g2_DrawLine(&u8g2, 50, 30, std::min(std::max(50 + az, 40), 60), 30);
        u8g2_DrawLine(&u8g2, 50, 30, 50, 31);
        xSemaphoreGive(display_mtx);
    };

    uint8_t width_tiles = u8g2_GetBufferTileWidth(&u8g2);
    uint8_t height_tiles = u8g2_GetBufferTileHeight(&u8g2);

    auto last_redraw = esp_timer_get_time();
    auto last_full_redraw = esp_timer_get_time();

    while (1) {
        check_log_start();
        redraw();
        redraw_gyro();
        u8g2_SendBuffer(&u8g2);
        last_full_redraw = esp_timer_get_time();

        for (int i = 0; i < 10; ++i) {
            while (esp_timer_get_time() - last_redraw < 100000) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                check_log_start();
            }
            check_log_start();
            redraw_gyro();
            u8g2_UpdateDisplayArea(&u8g2, 0, 3, 8, 1);
            last_redraw = esp_timer_get_time();
            if (last_redraw - last_full_redraw > 900000) {
                break;
            }
        }
    }
}

void work_atoms3() {
    spi_device_handle_t spi_device_;
    esp_lcd_panel_io_handle_t panel_io_;
    esp_lcd_panel_handle_t panel_;

    gpio_num_t gpio_dc = GPIO_NUM_33;
    gpio_num_t gpio_rst = GPIO_NUM_34;
    gpio_num_t gpio_bl = GPIO_NUM_16;
    gpio_num_t gpio_mosi = GPIO_NUM_21;
    gpio_num_t gpio_sclk = GPIO_NUM_17;
    gpio_num_t gpio_cs = GPIO_NUM_15;
    gpio_num_t gpios[] = {gpio_dc, gpio_rst, gpio_bl};
    for (int i = 0; i < sizeof(gpios) / sizeof(gpios[0]); ++i) {
        if (gpios[i] >= 0) {
            gpio_reset_pin(gpios[i]);
            gpio_set_direction(gpios[i], GPIO_MODE_OUTPUT);
            gpio_set_level(gpios[i], 0);
        }
    }
    if (gpio_rst >= 0) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(gpio_rst, 1);
    }

    // draw_sema = xSemaphoreCreateCounting(2, 2);

    spi_bus_config_t buscfg = {.mosi_io_num = gpio_mosi,
                               .miso_io_num = -1,
                               .sclk_io_num = gpio_sclk,
                               .quadwp_io_num = -1,
                               .quadhd_io_num = -1,
                               .max_transfer_sz = 0,
                               .flags = 0};

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = SPI_MASTER_FREQ_10M;
    devcfg.queue_size = 7;
    devcfg.mode = 3;
    devcfg.flags = SPI_DEVICE_NO_DUMMY;
    devcfg.spics_io_num = gpio_cs;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_device_));

    esp_lcd_panel_io_spi_config_t io_config = {.cs_gpio_num = gpio_cs,
                                               .dc_gpio_num = gpio_dc,
                                               .spi_mode = 0,
                                               .pclk_hz = 20000000,
                                               .trans_queue_depth = 4,
                                               .lcd_cmd_bits = 8,
                                               .lcd_param_bits = 8};
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &panel_io_));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = gpio_rst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io_, &panel_config, &panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_, 1, 2));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

    gpio_set_level(gpio_bl, 1);

    auto fill_display = [&](uint8_t r, uint8_t g, uint8_t b) {
        static uint8_t data[2 * 128 * 16] = {};
        for (int i = 0; i < 128 * 16; ++i) {
            data[2 * i + 0] = (b & 0x1f << 6) | ((g >> 3) & 0x7);
            data[2 * i + 1] = ((g & 0x7) << 5) | (r & 0x1f);
        }
        for (int k = 0; k < 128; k += 16) {
            esp_lcd_panel_draw_bitmap(panel_, 0, k, 128, k + 16, data);
        }
    };

    uint32_t duty{};
    while (true) {
        if (gctx.logger_control.busy &&
            (esp_timer_get_time() - gctx.logger_control.last_block_time_us <= 2000000ULL) &&
            !gctx.logger_control.storage_failure) {
            fill_display(255, 0, 0);
            gpio_set_level(gpio_bl, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(gpio_bl, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(gpio_bl, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(gpio_bl, 0);
            vTaskDelay(800 / portTICK_PERIOD_MS);
            duty = 0;
        } else {
            fill_display(0, 255 , 0);
            gpio_set_level(gpio_bl, duty >= 10);

            duty = (duty + 1) % 20;
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
    bool c = true;
}

void oled_capture(uint8_t *out) {
    if (!display_on) return;
    xSemaphoreTake(display_mtx, portMAX_DELAY);
    uint8_t *buf = u8g2_GetBufferPtr(&u8g2);
    uint8_t buf_w = u8g2_GetBufferTileWidth(&u8g2);
    uint8_t buf_h = u8g2_GetBufferTileHeight(&u8g2);

    int width_px = u8g2_GetDisplayWidth(&u8g2);
    int height_px = u8g2_GetDisplayHeight(&u8g2);

    for (int x = 0; x < width_px; ++x) {
        for (int y = 0; y < height_px; ++y) {
            int idx = x + width_px * y;
            if (u8x8_capture_get_pixel_1(x, y, buf, buf_w)) {
                out[(idx / 8) | 0] |= 1 << (idx % 8);
            } else {
                out[(idx / 8) | 0] &= ~(1 << (idx % 8));
            }
        }
    }
    xSemaphoreGive(display_mtx);
}

int oled_get_width() {
    if (!display_on) return 64;
    return u8g2_GetDisplayWidth(&u8g2);
}

int oled_get_height() {
    if (!display_on) return 32;
    return u8g2_GetDisplayHeight(&u8g2);
}

void display_setup() { display_mtx = xSemaphoreCreateMutex(); }

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
            display_on = true;
            work_64x32();
            break;
        case 2:  // ESP32-C3-0.42LCD
            u8g2_Setup_ssd1306_i2c_72x40_er_f(&u8g2, U8G2_R0, u8x8_byte_esplog,
                                              u8x8_gpio_and_delay_esplog);
            u8g2_SetI2CAddress(&u8g2, 0x78);
            u8g2_InitDisplay(&u8g2);
            u8g2_SetPowerSave(&u8g2, 0);
            display_on = true;
            work_64x32();
            break;
        case 4:  // minimal mode st7789 atoms3
            work_atoms3();
            break;
    }
}