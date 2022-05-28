#pragma once

#include <esp_check.h>

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_deinit(void);

esp_err_t i2c_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_register_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);