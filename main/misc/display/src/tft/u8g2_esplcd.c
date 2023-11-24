#include "u8g2_esplcd.h"
#include "u8x8.h"

#include <driver/gpio.h>

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdbool.h>
#include <string.h>

static const u8x8_display_info_t u8x8_d_st7789_135x240_display_info = {
    /* chip_enable_level = */ 0,
    /* chip_disable_level = */ 1,
    /* post_chip_enable_wait_ns = */ 120,
    /* pre_chip_disable_wait_ns = */ 60,
    /* reset_pulse_width_ms = */ 100,
    /* post_reset_wait_ms = */ 100,
    /* sda_setup_time_ns = */ 50,
    /* sck_pulse_width_ns = */ 125,
    /* sck_clock_hz = */ 4000000UL,
    /* spi_mode = */ 3, /* active high, rising edge */
    /* i2c_bus_clock_100kHz = */ 4,
    /* data_setup_time_ns = */ 40,
    /* write_pulse_width_ns = */ 150,
    /* tile_width = */ 17,
    /* tile_hight = */ 31,
    /* default_x_offset = */ 0,
    /* flipmode_x_offset = */ 0,
    /* pixel_width = */ 135,
    /* pixel_height = */ 240};

#define BYTES_PER_RGB_TILE 8192

static SemaphoreHandle_t draw_sema;

static bool draw_done_cb(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata,
                         void *user_ctx) {
    int hp_woken;
    xSemaphoreGiveFromISR(draw_sema, &hp_woken);
    return hp_woken;
}
static void u8x8_d_st7789_135x240_draw_tile(u8x8_t *u8x8, uint8_t arg_int, void *arg_ptr) {
    int x, y, c, w, h;
    uint8_t *ptr;
    static bool buf_id = 0;
    static uint8_t buf[BYTES_PER_RGB_TILE] = {0};
    uint8_t *rgb_pixels = buf + (buf_id ? 0 : 4096);
    buf_id = !buf_id;

    x = ((u8x8_tile_t *)arg_ptr)->x_pos;
    x = x * 8;
    y = (((u8x8_tile_t *)arg_ptr)->y_pos);
    y = y * 8;

    c = ((u8x8_tile_t *)arg_ptr)->cnt;
    w = 8 * c * arg_int;
    h = 8;

    ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;

    int pixels = w * h;

    // ESP_LOGI("st", "draw tile! %d %d %d %d %d", x, y, c, arg_int, pixels);

    u8g2_esplcd_t *user = (u8g2_esplcd_t *)u8x8->user_ptr;
    do {
        for (int i = 0; i < pixels; ++i) {
            rgb_pixels[2 * i] = (ptr[i / 8] & (1 << (i % 8))) ? 0xff : 0x00;
            rgb_pixels[2 * i + 1] = rgb_pixels[2 * i];
            // rgb_pixels[2 * i] = 0xff;
            // rgb_pixels[2 * i + 1] = 0xff;
        }
        xSemaphoreTake(draw_sema, portMAX_DELAY);
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(user->panel_, y, x, y + h, x + w, rgb_pixels));
        arg_int--;
    } while (arg_int > 0);
}

uint8_t u8x8_d_st7789_135x240(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DISPLAY_SETUP_MEMORY:
            u8x8_d_helper_display_setup_memory(u8x8, &u8x8_d_st7789_135x240_display_info);
            break;
        case U8X8_MSG_DISPLAY_DRAW_TILE:
            u8x8_d_st7789_135x240_draw_tile(u8x8, arg_int, arg_ptr);
            break;
        default:
            break;
    }
    return 1;
}

uint8_t *u8g2_m_17_31_f(uint8_t *page_cnt) {
#ifdef U8G2_USE_DYNAMIC_ALLOC
    *page_cnt = 31;
    return 0;
#else
    static uint8_t buf[4216];
    *page_cnt = 31;
    return buf;
#endif
}

void u8g2_Setup_st7789_135x240_f(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_msg_cb byte_cb,
                                 u8x8_msg_cb gpio_and_delay_cb) {
    uint8_t tile_buf_height;
    uint8_t *buf;
    u8g2_SetupDisplay(u8g2, u8x8_d_st7789_135x240, u8x8_cad_011, byte_cb, gpio_and_delay_cb);
    buf = u8g2_m_17_31_f(&tile_buf_height);
    u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}

int esplcd_init(u8g2_esplcd_t *user, int16_t gpio_mosi, int16_t gpio_sclk, int16_t gpio_cs,
                int16_t gpio_dc, int16_t gpio_rst, int16_t gpio_bl,
                int (*init_panel)(u8g2_esplcd_t *, int)) {
    int16_t gpios[] = {gpio_dc, gpio_rst, gpio_bl};
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

    draw_sema = xSemaphoreCreateCounting(2, 2);

    spi_bus_config_t buscfg = {.mosi_io_num = gpio_mosi,
                               .miso_io_num = -1,
                               .sclk_io_num = gpio_sclk,
                               .quadwp_io_num = -1,
                               .quadhd_io_num = -1,
                               .max_transfer_sz = 0,
                               .flags = 0};

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = SPI_MASTER_FREQ_10M;
    devcfg.queue_size = 7;
    devcfg.mode = 3;
    devcfg.flags = SPI_DEVICE_NO_DUMMY;
    devcfg.spics_io_num = gpio_cs;
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &user->spi_device_));

    esp_lcd_panel_io_spi_config_t io_config = {.dc_gpio_num = gpio_dc,
                                               .cs_gpio_num = gpio_cs,
                                               .pclk_hz = 5000000,
                                               .lcd_cmd_bits = 8,
                                               .lcd_param_bits = 8,
                                               .spi_mode = 3,
                                               .trans_queue_depth = 1,
                                               .on_color_trans_done = draw_done_cb};
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)HSPI_HOST, &io_config,
                                             &user->panel_io_));

    if ((*init_panel)(user, gpio_rst)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static uint8_t u8x8_gpio_and_delay_dummy(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                                         void *arg_ptr) {
    return 1;
}

static uint8_t u8x8_byte_dummy(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    return 1;
}

int init_panel_st7789_m5stickc_plus(u8g2_esplcd_t *user, int gpio_reset) {
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = gpio_reset,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(user->panel_io_, &panel_config, &user->panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(user->panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_init(user->panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(user->panel_, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(user->panel_, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(user->panel_, 52, 52));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(user->panel_, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(user->panel_, true));

    u8g2_Setup_st7789_135x240_f(user->u8g2, &u8g2_cb_r0, u8x8_byte_dummy,
                                u8x8_gpio_and_delay_dummy);
    user->u8g2->u8x8.user_ptr = user;
    u8g2_InitDisplay(user->u8g2);
    return 0;
}