#pragma once

#include <esp_err.h>

#include <stdint.h>

esp_err_t mini_i2c_init(int sda_pin, int scl_pin, int freq);


esp_err_t mini_i2c_read_reg_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t* bytes, uint8_t n_bytes);
esp_err_t mini_i2c_write_reg_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t byte);


esp_err_t mini_i2c_read_reg_callback(uint8_t dev_adr, uint8_t reg_adr, uint8_t n_bytes,
                                void (*callback)(void* args), void* callback_args);
esp_err_t mini_i2c_read_reg_get_result(uint8_t* bytes, uint8_t n_bytes);

esp_err_t mini_i2c_hw_fsm_reset();