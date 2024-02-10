#pragma once
#include <freertos/FreeRTOS.h>

#include "soc/clk_tree_defs.h"
#include "soc/i2c_struct.h"

typedef enum {
    I2C_STATUS_IDLE,
    I2C_STATUS_TIMEOUT,
    I2C_STATUS_ARB_LOST,
    I2C_STATUS_NACK,
    I2C_STATUS_ACTIVE
} i2c_status_t;

struct i2c_ctx {
    int sda;
    int scl;
    uint32_t i2c_freq;
    int i2c_port;
    i2c_dev_t *dev;
    soc_periph_i2c_clk_src_t clk_src;
    uint32_t periph_src_clk_hz;
    volatile i2c_status_t status;
    void (*callback)(void *args);
    void *callback_args;
    SemaphoreHandle_t mtx;
    intr_handle_t int_hndl;
};

esp_err_t mini_i2c_init(struct i2c_ctx *ctx, int sda, int scl, uint32_t i2c_freq, int i2c_port);
esp_err_t mini_i2c_hw_fsm_reset(struct i2c_ctx *ctx);

esp_err_t mini_i2c_read_reg_sync(struct i2c_ctx *ctx, uint8_t dev_adr, uint8_t reg_adr,
                                 uint8_t *bytes, uint8_t n_bytes);

esp_err_t mini_i2c_read_reg_callback(struct i2c_ctx *ctx, uint8_t dev_adr, uint8_t reg_adr,
                                     uint8_t n_bytes, void (*callback)(void *args),
                                     void *callback_args);

esp_err_t mini_i2c_read_reg_get_result(struct i2c_ctx *ctx, uint8_t *bytes, uint8_t n_bytes);

esp_err_t mini_i2c_write_reg_sync(struct i2c_ctx *ctx, uint8_t dev_adr, uint8_t reg_adr,
                                  uint8_t byte);

esp_err_t mini_i2c_write_reg2_sync(struct i2c_ctx *ctx, uint8_t dev_adr, uint8_t reg_adr,
                                   uint8_t byte1, uint8_t byte2);

esp_err_t mini_i2c_write_n_sync(struct i2c_ctx *ctx, uint8_t *data, int len);

esp_err_t mini_i2c_double_stop_timing(struct i2c_ctx *ctx);

esp_err_t mini_i2c_set_timing(struct i2c_ctx *ctx, uint32_t i2c_freq);