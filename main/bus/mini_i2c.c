#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_hal.h"
#include "mini_i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include "driver/i2c_private.h"

#define TAG "i2c"



typedef struct {
    i2c_bus_handle_t handle;
    intr_handle_t int_hndl;
    _Atomic minii2c_status status;
    int sda_pin, scl_pin;
    void (*callback)(void* args);
    void* callback_args;
    SemaphoreHandle_t mtx;
} i2c_ctx_t;

i2c_ctx_t i2c_ctx;

/* IRAM-safe */

minii2c_status IRAM_ATTR mini_i2c_get_status() { return i2c_ctx.status; }

static void IRAM_ATTR i2c_isr_handler(void* arg) {
    i2c_ctx_t* ctx = (i2c_ctx_t*)arg;
    // i2c_ctx_t* ctx = &i2c_ctx;
    uint32_t int_mask;
    i2c_ll_get_intr_mask(ctx->handle->hal.dev, &int_mask);
    i2c_ll_clear_intr_mask(ctx->handle->hal.dev, int_mask);

    if (int_mask == 0) {
        return;
    }

    if (int_mask == I2C_LL_INTR_NACK) {
        atomic_store(&ctx->status, MINII2C_STATUS_NACK);
    } else if (int_mask == I2C_LL_INTR_TIMEOUT || int_mask == I2C_LL_INTR_ARBITRATION) {
        atomic_store(&ctx->status, MINII2C_STATUS_TIMEOUT);
    } else if (int_mask == I2C_LL_INTR_MST_COMPLETE) {
        atomic_store(&ctx->status, MINII2C_STATUS_IDLE);
    } else {
        return;
    }

    if (ctx->callback) {
        void (*callback)(void* args);
        callback = ctx->callback;
        ctx->callback = NULL;
        callback(ctx->callback_args);
    }
}

esp_err_t IRAM_ATTR mini_i2c_read_reg_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t* bytes,
                                           uint8_t n_bytes) {
    if (!xSemaphoreTakeFromISR(i2c_ctx.mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }

    i2c_hal_context_t hal = i2c_ctx.handle->hal;

    i2c_ll_rxfifo_rst(hal.dev);
    i2c_ll_txfifo_rst(hal.dev);

    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, (dev_adr << 1) | 1};
    i2c_ll_write_txfifo(hal.dev, data, 3);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 2, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 1, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    if (n_bytes > 1) {
        i2c_ll_hw_cmd_t hw_cmd = {
            .op_code = I2C_LL_CMD_READ, .byte_num = n_bytes - 1, .ack_val = 0};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_READ, .byte_num = 1, .ack_val = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }

    atomic_store(&i2c_ctx.status, MINII2C_STATUS_ACTIVE);
    i2c_ll_master_enable_tx_it(hal.dev);
    i2c_ll_master_trans_start(hal.dev);

    int cnt = 1000;
    while (cnt && atomic_load(&i2c_ctx.status) == MINII2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(&i2c_ctx);
        --cnt;
    }

    if (!cnt) {
        xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
        return ESP_FAIL;
    }

    i2c_ll_read_rxfifo(hal.dev, bytes, n_bytes);

    if (i2c_ctx.status != MINII2C_STATUS_IDLE) {
        xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
        return ESP_FAIL;
    }
    xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_read_reg_callback(uint8_t dev_adr, uint8_t reg_adr, uint8_t n_bytes,
                                               void (*callback)(void* args), void* callback_args) {
    if (!xSemaphoreTakeFromISR(i2c_ctx.mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }

    i2c_hal_context_t hal = i2c_ctx.handle->hal;

    i2c_ll_rxfifo_rst(hal.dev);
    i2c_ll_txfifo_rst(hal.dev);

    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, (dev_adr << 1) | 1};
    i2c_ll_write_txfifo(hal.dev, data, 3);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 2, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 1, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    if (n_bytes > 1) {
        i2c_ll_hw_cmd_t hw_cmd = {
            .op_code = I2C_LL_CMD_READ, .byte_num = n_bytes - 1, .ack_val = 0};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_READ, .byte_num = 1, .ack_val = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }

    i2c_ctx.callback = callback;
    i2c_ctx.callback_args = callback_args;
    atomic_store(&i2c_ctx.status, MINII2C_STATUS_ACTIVE);
    i2c_ll_master_enable_tx_it(hal.dev);
    i2c_ll_master_trans_start(hal.dev);
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_read_reg_get_result(uint8_t* bytes, uint8_t n_bytes) {
    i2c_hal_context_t hal = i2c_ctx.handle->hal;
    i2c_ll_read_rxfifo(hal.dev, bytes, n_bytes);

    if (i2c_ctx.status != MINII2C_STATUS_IDLE) {
        xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
        return ESP_FAIL;
    }

    xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_write_reg_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t byte) {
    if (!xSemaphoreTakeFromISR(i2c_ctx.mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }

    i2c_hal_context_t hal = i2c_ctx.handle->hal;

    i2c_ll_rxfifo_rst(hal.dev);
    i2c_ll_txfifo_rst(hal.dev);

    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, byte};
    i2c_ll_write_txfifo(hal.dev, data, 3);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 3, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }

    atomic_store(&i2c_ctx.status, MINII2C_STATUS_ACTIVE);
    i2c_ll_master_enable_tx_it(hal.dev);
    i2c_ll_master_trans_start(hal.dev);

    while (atomic_load(&i2c_ctx.status) == MINII2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(&i2c_ctx);
    }

    if (i2c_ctx.status != MINII2C_STATUS_IDLE) {
        xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
        return ESP_FAIL;
    }

    xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_write_reg2_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t byte1,
                                             uint8_t byte2) {
    if (!xSemaphoreTakeFromISR(i2c_ctx.mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }

    i2c_hal_context_t hal = i2c_ctx.handle->hal;

    i2c_ll_rxfifo_rst(hal.dev);
    i2c_ll_txfifo_rst(hal.dev);

    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, byte1, byte2};
    i2c_ll_write_txfifo(hal.dev, data, 4);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 4, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }

    atomic_store(&i2c_ctx.status, MINII2C_STATUS_ACTIVE);
    i2c_ll_master_enable_tx_it(hal.dev);
    i2c_ll_master_trans_start(hal.dev);

    while (atomic_load(&i2c_ctx.status) == MINII2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(&i2c_ctx);
    }

    if (i2c_ctx.status != MINII2C_STATUS_IDLE) {
        xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
        return ESP_FAIL;
    }

    xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_write_n_sync(uint8_t* data, int len) {
    if (!xSemaphoreTakeFromISR(i2c_ctx.mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }

    i2c_hal_context_t hal = i2c_ctx.handle->hal;

    i2c_ll_rxfifo_rst(hal.dev);
    i2c_ll_txfifo_rst(hal.dev);

    int idx = 0;
    i2c_ll_write_txfifo(hal.dev, data, len);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = len, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(hal.dev, hw_cmd, idx++);
    }

    atomic_store(&i2c_ctx.status, MINII2C_STATUS_ACTIVE);
    i2c_ll_master_enable_tx_it(hal.dev);
    i2c_ll_master_trans_start(hal.dev);

    while (atomic_load(&i2c_ctx.status) == MINII2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(&i2c_ctx);
    }

    if (i2c_ctx.status != MINII2C_STATUS_IDLE) {
        xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
        return ESP_FAIL;
    }

    xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
    return ESP_OK;
}

/* Not IRAM-safe */

static esp_err_t mini_i2c_master_clear_bus(i2c_bus_handle_t handle) {
#if !SOC_I2C_SUPPORT_HW_FSM_RST
    const int scl_half_period = 5;  // use standard 100kHz data rate
    int i = 0;
    gpio_set_direction(handle->scl_num, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(handle->sda_num, GPIO_MODE_INPUT_OUTPUT_OD);
    // If a SLAVE device was in a read operation when the bus was interrupted, the SLAVE device
    // is controlling SDA. The only bit during the 9 clock cycles of a READ byte the
    // MASTER(ESP32) is guaranteed control over is during the ACK bit period. If the slave is
    // sending a stream of ZERO bytes, it will only release SDA during the ACK bit period. So,
    // this reset code needs to synchronous the bit stream with, Either, the ACK bit, Or a 1 bit
    // to correctly generate a STOP condition.
    gpio_set_level(handle->scl_num, 0);
    gpio_set_level(handle->sda_num, 1);
    esp_rom_delay_us(scl_half_period);
    while (!gpio_get_level(handle->sda_num) && (i++ < 9)) {
        gpio_set_level(handle->scl_num, 1);
        esp_rom_delay_us(scl_half_period);
        gpio_set_level(handle->scl_num, 0);
        esp_rom_delay_us(scl_half_period);
    }
    gpio_set_level(handle->sda_num, 0);  // setup for STOP
    gpio_set_level(handle->scl_num, 1);
    esp_rom_delay_us(scl_half_period);
    gpio_set_level(handle->sda_num, 1);  // STOP, SDA low -> high while SCL is HIGH
    i2c_common_set_pins(handle);
#else
    i2c_hal_context_t* hal = &handle->hal;
    i2c_ll_master_clr_bus(hal->dev, I2C_LL_RESET_SLV_SCL_PULSE_NUM_DEFAULT);
#endif
    return ESP_OK;
}

extern void periph_module_enable(periph_module_t periph);

extern void periph_module_disable(periph_module_t periph);

esp_err_t mini_i2c_hw_fsm_reset() {
    i2c_bus_handle_t handle = i2c_ctx.handle;
    i2c_hal_context_t* hal = &handle->hal;
    i2c_hal_timing_config_t timing_config;
    uint8_t filter_cfg;

    esp_intr_disable(handle->intr_handle);
    i2c_hal_get_timing_config(hal, &timing_config);
    i2c_ll_master_get_filter(hal->dev, &filter_cfg);

    // to reset the I2C hw module, we need re-enable the hw
    mini_i2c_master_clear_bus(handle);
    periph_module_disable(i2c_periph_signal[handle->port_num].module);
    periph_module_enable(i2c_periph_signal[handle->port_num].module);

    i2c_hal_master_init(hal);
    i2c_ll_disable_intr_mask(hal->dev, I2C_LL_INTR_MASK);
    i2c_ll_clear_intr_mask(hal->dev, I2C_LL_INTR_MASK);
    i2c_hal_set_timing_config(hal, &timing_config);
    i2c_ll_master_set_filter(hal->dev, filter_cfg);
    esp_intr_enable(handle->intr_handle);
    return ESP_OK;
}

esp_err_t mini_i2c_double_stop_timing() {
    int scl_stop_hold, scl_stop_setup;
    i2c_ll_get_stop_timing(i2c_ctx.handle->hal.dev, &scl_stop_setup, &scl_stop_hold);
    scl_stop_hold *= 2;
    i2c_ll_master_set_stop_timing(i2c_ctx.handle->hal.dev, scl_stop_setup, scl_stop_hold);
    i2c_ll_update(i2c_ctx.handle->hal.dev);
    return ESP_OK;
}

esp_err_t mini_i2c_set_timing(int freq) {
    i2c_hal_context_t *hal = &i2c_ctx.handle->hal;
    i2c_hal_set_bus_timing(hal, freq, i2c_ctx.handle->clk_src, i2c_ctx.handle->clk_src_freq_hz);
    i2c_ll_master_set_fractional_divider(hal->dev, 0, 0);
    i2c_ll_update(hal->dev);

    mini_i2c_hw_fsm_reset(i2c_ctx.handle);
    return ESP_OK;
}

esp_err_t mini_i2c_init(int sda_pin, int scl_pin, int freq) {
    i2c_bus_handle_t handle;
    ESP_ERROR_CHECK(i2c_acquire_bus_handle(0, &handle, I2C_BUS_MODE_MASTER));

    /* setup i2c peripheral */
    handle->sda_num = sda_pin;
    handle->scl_num = scl_pin;
    i2c_hal_context_t* hal = &handle->hal;
    ESP_ERROR_CHECK(i2c_common_set_pins(handle));
    ESP_ERROR_CHECK(i2c_select_periph_clock(handle, I2C_CLK_SRC_DEFAULT));
    handle->clk_src = I2C_CLK_SRC_DEFAULT;
    i2c_hal_master_init(hal);
    i2c_ll_update(hal->dev);

    i2c_ll_clear_intr_mask(hal->dev, I2C_LL_MASTER_EVENT_INTR);

    i2c_ctx.handle = handle;
    i2c_ctx.mtx = xSemaphoreCreateMutex();
    i2c_ctx.status = MINII2C_STATUS_IDLE;

    intr_handle_t intr_handle;
    esp_intr_alloc(i2c_periph_signal[0].irq, ESP_INTR_FLAG_IRAM, i2c_isr_handler, &i2c_ctx,
                   &intr_handle);

    // i2c_ll_enable_intr_mask(hal->dev, I2C_LL_MASTER_EVENT_INTR);
    i2c_ll_master_set_filter(hal->dev, 0);

    i2c_hal_set_bus_timing(hal, freq, handle->clk_src, handle->clk_src_freq_hz);
    i2c_ll_master_set_fractional_divider(hal->dev, 0, 0);
    i2c_ll_update(hal->dev);

    mini_i2c_hw_fsm_reset();

    return ESP_OK;
}