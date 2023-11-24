#pragma once
#include "u8g2.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_ops.h"

typedef struct {
    u8g2_t* u8g2;
    spi_device_handle_t spi_device_;
    esp_lcd_panel_io_handle_t panel_io_;
    esp_lcd_panel_handle_t panel_;
} u8g2_esplcd_t;

int esplcd_init(u8g2_esplcd_t *user, int16_t gpio_mosi, int16_t gpio_sclk, int16_t gpio_cs,
                int16_t gpio_dc, int16_t gpio_rst, int16_t gpio_bl,
                int (*init_panel)(u8g2_esplcd_t *, int));
int init_panel_st7789_m5stickc_plus(u8g2_esplcd_t *user, int gpio_reset);