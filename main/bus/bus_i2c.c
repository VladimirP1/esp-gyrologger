// SPDX-License-Identifier: LGPL-2.1-or-later

#include "bus_i2c.h"

#include <hal/gpio_ll.h>
#include <driver/gpio.h>

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_NUM CONFIG_I2C_MASTER_NUM
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQ_HZ
#define I2C_MASTER_TIMEOUT_MS 1000

#if CONFIG_IDF_TARGET_ESP32C3
static inline bool G_SDA() { return GPIO.in.data & (1 << I2C_MASTER_SDA_IO); }

static inline void SDA(bool x) {
    if (x) {
        GPIO.out_w1ts.out_w1ts = (1 << I2C_MASTER_SDA_IO);
    } else {
        GPIO.out_w1tc.out_w1tc = (1 << I2C_MASTER_SDA_IO);
    }
}

static inline void SCL(bool x) {
    if (x) {
        GPIO.out_w1ts.out_w1ts = (1 << I2C_MASTER_SCL_IO);
    } else {
        GPIO.out_w1tc.out_w1tc = (1 << I2C_MASTER_SCL_IO);
    }
}

static int hclk_top = 10;
#define HCLK()                           \
    for (int i = 0; i < hclk_top; i++) { \
        __asm__("nop");                  \
    };

#elif CONFIG_IDF_TARGET_ESP32
static inline bool G_SDA() { return GPIO.in & (1 << I2C_MASTER_SDA_IO); }

static inline void SDA(bool x) {
    if (x) {
        GPIO.out_w1ts = (1 << I2C_MASTER_SDA_IO);
    } else {
        GPIO.out_w1tc = (1 << I2C_MASTER_SDA_IO);
    }
}

static inline void SCL(bool x) {
    if (x) {
        GPIO.out_w1ts = (1 << I2C_MASTER_SCL_IO);
    } else {
        GPIO.out_w1tc = (1 << I2C_MASTER_SCL_IO);
    }
}

static int hclk_top = 10;
#define HCLK()                     \
    for (int i = 0; i < hclk_top; i++) { \
        __asm__("nop");            \
    };

#endif

#define CLK() \
    HCLK();   \
    HCLK();

esp_err_t i2c_master_init() {
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);

    gpio_config_t io_conf = {.intr_type = GPIO_INTR_DISABLE,
                             .mode = GPIO_MODE_INPUT_OUTPUT,
                             .pin_bit_mask = (1 << I2C_MASTER_SCL_IO) | (1 << I2C_MASTER_SDA_IO),
                             .pull_up_en = GPIO_PULLUP_ENABLE};

    gpio_config(&io_conf);

    gpio_set_drive_capability(I2C_MASTER_SCL_IO, GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(I2C_MASTER_SDA_IO, GPIO_DRIVE_CAP_0);

    return ESP_OK;
}

void i2c_set_overclock(bool enable) { hclk_top = enable ? 0 : 10; }

static void IRAM_ATTR i2c_bb_start() {
    SDA(1);
    SCL(1);
    HCLK();
    SDA(0);
    HCLK();
    SCL(0);
}

static uint8_t IRAM_ATTR i2c_bb_rcvbyte(uint8_t ack) {
    uint8_t data = 0;
    SDA(1);
    for (uint8_t i = 0; i < 8; ++i) {
        data <<= 1;
        CLK();
        SCL(1);
        CLK();
        data |= G_SDA() ? 1 : 0;
        SCL(0);
    }
    HCLK();
    SDA(!ack);
    HCLK();
    SCL(1);
    CLK();
    SCL(0);
    CLK();
    return data;
}

static uint8_t IRAM_ATTR i2c_bb_sendbyte(uint8_t data) {
    for (uint8_t i = 0; i < 8; ++i) {
        HCLK();
        SDA(data & 0x80);
        HCLK();
        SCL(1);
        CLK();
        SCL(0);
        data <<= 1;
    }
    HCLK();
    SDA(1);
    HCLK();
    SCL(1);
    CLK();
    uint8_t ack = !G_SDA();
    SCL(0);
    CLK();
    return ack;
}

static void IRAM_ATTR i2c_bb_stop() {
    SDA(0);
    HCLK();
    SCL(1);
    HCLK();
    SDA(1);
    HCLK();
}

esp_err_t IRAM_ATTR i2c_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                                      size_t len) {
    i2c_bb_start();
    if (!i2c_bb_sendbyte(dev_addr << 1)) {  // WRITE
        return ESP_FAIL;
    }

    if (!i2c_bb_sendbyte(reg_addr)) {
        return ESP_FAIL;
    }
    if (len) {
        i2c_bb_start();
        if (!i2c_bb_sendbyte(dev_addr << 1 | 1)) {  // READ
            return ESP_FAIL;
        }
        for (size_t i = 0; i < len; ++i) {
            data[i] = i2c_bb_rcvbyte(i != len - 1);
        }
    }
    i2c_bb_stop();

    return ESP_OK;
}

esp_err_t IRAM_ATTR i2c_register_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_bb_start();
    if (!i2c_bb_sendbyte(dev_addr << 1)) {  // WRITE
        return ESP_FAIL;
    }
    if (!i2c_bb_sendbyte(reg_addr)) {
        return ESP_FAIL;
    }
    if (!i2c_bb_sendbyte(data)) {
        return ESP_FAIL;
    }
    i2c_bb_stop();

    return ESP_OK;
}

esp_err_t i2c_master_deinit() { return ESP_OK; }