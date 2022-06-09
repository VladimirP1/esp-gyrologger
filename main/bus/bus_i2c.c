#include "bus_i2c.h"

#include <hal/gpio_ll.h>
#include <driver/gpio.h>

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_NUM CONFIG_I2C_MASTER_NUM
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQ_HZ
#define I2C_MASTER_TIMEOUT_MS 1000

#define G_SDA() gpio_get_level(I2C_MASTER_SDA_IO)
#define SDA(x) gpio_set_level(I2C_MASTER_SDA_IO, (x));
#define SCL(x) gpio_set_level(I2C_MASTER_SCL_IO, (x));

#define HCLK()                   \
    for (int i = 0; i < 8; i++) \
    {                            \
        __asm__("nop");          \
    };
#define CLK() \
    HCLK();   \
    HCLK();

esp_err_t i2c_master_init()
{
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pin_bit_mask = (1 << I2C_MASTER_SCL_IO) | (1 << I2C_MASTER_SDA_IO),
        .pull_up_en = GPIO_PULLUP_ENABLE};

    gpio_config(&io_conf);

    return ESP_OK;
}

static void IRAM_ATTR i2c_bb_start()
{
    SDA(1);
    SCL(1);
    HCLK();
    SDA(0);
    HCLK();
    SCL(0);
}

static uint8_t IRAM_ATTR i2c_bb_rcvbyte(uint8_t ack)
{
    uint8_t data = 0;
    SDA(1);
    for (uint8_t i = 0; i < 8; ++i)
    {
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

static uint8_t IRAM_ATTR i2c_bb_sendbyte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; ++i)
    {
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

static void IRAM_ATTR i2c_bb_stop()
{
    SDA(0);
    HCLK();
    SCL(1);
    HCLK();
    SDA(1);
    HCLK();
}

esp_err_t IRAM_ATTR i2c_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{

    i2c_bb_start();
    if (!i2c_bb_sendbyte(dev_addr << 1))
    { // WRITE
        return ESP_FAIL;
    }

    if (!i2c_bb_sendbyte(reg_addr))
    {
        return ESP_FAIL;
    }
    if (len)
    {
        i2c_bb_start();
        if (!i2c_bb_sendbyte(dev_addr << 1 | 1))
        { // READ
            return ESP_FAIL;
        }
        for (size_t i = 0; i < len; ++i)
        {
            data[i] = i2c_bb_rcvbyte(i != len - 1);
        }
    }
    i2c_bb_stop();

    return ESP_OK;
}

esp_err_t IRAM_ATTR i2c_register_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_bb_start();
    if (!i2c_bb_sendbyte(dev_addr << 1))
    { // WRITE
        return ESP_FAIL;
    }
    if (!i2c_bb_sendbyte(reg_addr))
    {
        return ESP_FAIL;
    }
    if (!i2c_bb_sendbyte(data))
    {
        return ESP_FAIL;
    }
    i2c_bb_stop();

    return ESP_OK;
}

esp_err_t i2c_master_deinit()
{
    return ESP_OK;
}