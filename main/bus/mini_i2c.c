#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_hal.h"
#include "hal/i2c_hal.h"
#include "mini_i2c.h"
#include "soc/i2c_periph.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "soc/soc.h"
#include <stdio.h>

#define TAG "i2c"

void periph_module_enable(periph_module_t periph);
void periph_module_disable(periph_module_t periph);

typedef enum { I2C_STATUS_IDLE, I2C_STATUS_FAIL, I2C_STATUS_ACTIVE } i2c_status_t;

typedef struct {
    i2c_hal_context_t hal;
    intr_handle_t int_hndl;
    volatile i2c_status_t status;
    int sda_pin, scl_pin;
    void (*callback)(void* args);
    void* callback_args;
    SemaphoreHandle_t mtx;
} i2c_ctx_t;

i2c_ctx_t i2c_ctx;

static void IRAM_ATTR i2c_isr_handler(void* arg) {
    uint32_t int_mask;
    i2c_hal_get_intsts_mask(&i2c_ctx.hal, &int_mask);
    if (int_mask == 0) {
        return;
    }

    i2c_intr_event_t evt_type = I2C_INTR_EVENT_ERR;
    if (i2c_ctx.status == I2C_STATUS_ACTIVE) {
        i2c_hal_master_handle_tx_event(&i2c_ctx.hal, &evt_type);
    }

    if (evt_type == I2C_INTR_EVENT_NACK) {
        i2c_ctx.status = I2C_STATUS_FAIL;
    } else if (evt_type == I2C_INTR_EVENT_TOUT) {
        i2c_ctx.status = I2C_STATUS_FAIL;
    } else if (evt_type == I2C_INTR_EVENT_ARBIT_LOST) {
        i2c_ctx.status = I2C_STATUS_FAIL;
    } else if (evt_type == I2C_INTR_EVENT_END_DET) {
        i2c_ctx.status = I2C_STATUS_IDLE;
    } else if (evt_type == I2C_INTR_EVENT_TRANS_DONE) {
        i2c_ctx.status = I2C_STATUS_IDLE;
    } else {
        return;
    }
    if (i2c_ctx.callback) {
        i2c_ctx.callback(i2c_ctx.callback_args);
        i2c_ctx.callback = NULL;
    }
}

static inline void mini_i2c_write_txfifo(i2c_hal_context_t* hal, uint8_t* ptr, uint8_t len) {
    // TODO: handle i2c 0/1
    uint32_t fifo_addr = 0x6001301c;
    for (int i = 0; i < len; i++) {
        WRITE_PERI_REG(fifo_addr, ptr[i]);
    }
}

static esp_err_t i2c_conf_pins(i2c_port_t i2c_num, int sda_io_num, int scl_io_num,
                               bool sda_pullup_en, bool scl_pullup_en) {
    int sda_in_sig, sda_out_sig, scl_in_sig, scl_out_sig;
    sda_out_sig = i2c_periph_signal[i2c_num].sda_out_sig;
    sda_in_sig = i2c_periph_signal[i2c_num].sda_in_sig;
    scl_out_sig = i2c_periph_signal[i2c_num].scl_out_sig;
    scl_in_sig = i2c_periph_signal[i2c_num].scl_in_sig;
    if (sda_io_num >= 0) {
        gpio_set_level(sda_io_num, 1);
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[sda_io_num], PIN_FUNC_GPIO);
        gpio_set_direction(sda_io_num, GPIO_MODE_INPUT_OUTPUT_OD);

        if (sda_pullup_en == GPIO_PULLUP_ENABLE) {
            gpio_set_pull_mode(sda_io_num, GPIO_PULLUP_ONLY);
        } else {
            gpio_set_pull_mode(sda_io_num, GPIO_FLOATING);
        }
        esp_rom_gpio_connect_out_signal(sda_io_num, sda_out_sig, 0, 0);
        esp_rom_gpio_connect_in_signal(sda_io_num, sda_in_sig, 0);
    }
    if (scl_io_num >= 0) {
        gpio_set_level(scl_io_num, 1);
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[scl_io_num], PIN_FUNC_GPIO);
        gpio_set_direction(scl_io_num, GPIO_MODE_INPUT_OUTPUT_OD);
        esp_rom_gpio_connect_out_signal(scl_io_num, scl_out_sig, 0, 0);
        esp_rom_gpio_connect_in_signal(scl_io_num, scl_in_sig, 0);
        if (scl_pullup_en == GPIO_PULLUP_ENABLE) {
            gpio_set_pull_mode(scl_io_num, GPIO_PULLUP_ONLY);
        } else {
            gpio_set_pull_mode(scl_io_num, GPIO_FLOATING);
        }
    }
    i2c_ctx.sda_pin = sda_io_num;
    i2c_ctx.scl_pin = scl_io_num;
    return ESP_OK;
}

esp_err_t mini_i2c_init(int sda_pin, int scl_pin, int freq) {
    esp_err_t err;
    i2c_ctx.mtx = xSemaphoreCreateMutex();
    i2c_ctx.hal.dev = I2C_LL_GET_HW((0));
    i2c_ctx.status = I2C_STATUS_IDLE;
    if ((err = i2c_conf_pins(0, sda_pin, scl_pin, 1, 1) != ESP_OK)) {
        return err;
    }
    periph_module_enable(i2c_periph_signal[0].module);
    i2c_hal_disable_intr_mask(&i2c_ctx.hal, I2C_LL_INTR_MASK);
    i2c_hal_clr_intsts_mask(&i2c_ctx.hal, I2C_LL_INTR_MASK);
    i2c_hal_master_init(&i2c_ctx.hal, 0);
    i2c_hal_set_filter(&i2c_ctx.hal, 7);
    i2c_hal_set_tout(&i2c_ctx.hal, 10);
    i2c_hal_set_bus_timing(&i2c_ctx.hal, freq, I2C_SCLK_APB);
    i2c_hal_update_config(&i2c_ctx.hal);
    i2c_hal_disable_intr_mask(&i2c_ctx.hal, I2C_LL_INTR_MASK);
    i2c_hal_clr_intsts_mask(&i2c_ctx.hal, I2C_LL_INTR_MASK);
    esp_intr_alloc(i2c_periph_signal[0].irq, ESP_INTR_FLAG_IRAM, i2c_isr_handler, NULL,
                   &i2c_ctx.int_hndl);
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_read_reg_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t* bytes,
                                           uint8_t n_bytes) {
    if (!xSemaphoreTakeFromISR(i2c_ctx.mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }
    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, (dev_adr << 1) | 1};
    mini_i2c_write_txfifo(&i2c_ctx.hal, data, 3);
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 2, .ack_en = 1};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 1, .ack_en = 1};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    if (n_bytes > 1) {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_READ, .byte_num = n_bytes - 1, .ack_val = 0};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_READ, .byte_num = 1, .ack_val = 1};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }

    i2c_ctx.status = I2C_STATUS_ACTIVE;
    i2c_hal_enable_master_tx_it(&i2c_ctx.hal);
    i2c_hal_trans_start(&i2c_ctx.hal);

    while (i2c_ctx.status == I2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(NULL);
    }

    i2c_hal_read_rxfifo(&i2c_ctx.hal, bytes, n_bytes);
    i2c_hal_rxfifo_rst(&i2c_ctx.hal);
    i2c_hal_txfifo_rst(&i2c_ctx.hal);

    if (i2c_ctx.status != I2C_STATUS_IDLE) {
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
    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, (dev_adr << 1) | 1};
    mini_i2c_write_txfifo(&i2c_ctx.hal, data, 3);
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 2, .ack_en = 1};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 1, .ack_en = 1};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    if (n_bytes > 1) {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_READ, .byte_num = n_bytes - 1, .ack_val = 0};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_READ, .byte_num = 1, .ack_val = 1};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }

    i2c_ctx.callback = callback;
    i2c_ctx.callback_args = callback_args;
    i2c_ctx.status = I2C_STATUS_ACTIVE;
    i2c_hal_enable_master_tx_it(&i2c_ctx.hal);
    i2c_hal_trans_start(&i2c_ctx.hal);

    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_read_reg_get_result(uint8_t* bytes, uint8_t n_bytes) {
    i2c_hal_read_rxfifo(&i2c_ctx.hal, bytes, n_bytes);
    i2c_hal_rxfifo_rst(&i2c_ctx.hal);
    i2c_hal_txfifo_rst(&i2c_ctx.hal);

    if (i2c_ctx.status != I2C_STATUS_IDLE) {
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
    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, byte};
    mini_i2c_write_txfifo(&i2c_ctx.hal, data, 3);
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 3, .ack_en = 1};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }
    {
        i2c_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_hal_write_cmd_reg(&i2c_ctx.hal, hw_cmd, idx++);
    }

    i2c_ctx.status = I2C_STATUS_ACTIVE;
    i2c_hal_enable_master_tx_it(&i2c_ctx.hal);
    i2c_hal_trans_start(&i2c_ctx.hal);

    while (i2c_ctx.status == I2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(NULL);
    }

    i2c_hal_rxfifo_rst(&i2c_ctx.hal);
    i2c_hal_txfifo_rst(&i2c_ctx.hal);

    if (i2c_ctx.status != I2C_STATUS_IDLE) {
        xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
        return ESP_FAIL;
    }

    xSemaphoreGiveFromISR(i2c_ctx.mtx, NULL);
    return ESP_OK;
}

static esp_err_t mini_i2c_master_clear_bus(i2c_port_t i2c_num) {
#if !SOC_I2C_SUPPORT_HW_CLR_BUS
    const int scl_half_period = 5;  // use standard 100kHz data rate
    int i = 0;
    int scl_io = i2c_ctx.scl_pin;
    int sda_io = i2c_ctx.sda_pin;
    gpio_set_direction(scl_io, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(sda_io, GPIO_MODE_INPUT_OUTPUT_OD);
    // If a SLAVE device was in a read operation when the bus was interrupted, the SLAVE device is
    // controlling SDA. The only bit during the 9 clock cycles of a READ byte the MASTER(ESP32) is
    // guaranteed control over is during the ACK bit period. If the slave is sending a stream of
    // ZERO bytes, it will only release SDA during the ACK bit period. So, this reset code needs to
    // synchronize the bit stream with, Either, the ACK bit, Or a 1 bit to correctly generate a STOP
    // condition.
    gpio_set_level(scl_io, 0);
    gpio_set_level(sda_io, 1);
    esp_rom_delay_us(scl_half_period);
    while (!gpio_get_level(sda_io) && (i++ < 9)) {
        gpio_set_level(scl_io, 1);
        esp_rom_delay_us(scl_half_period);
        gpio_set_level(scl_io, 0);
        esp_rom_delay_us(scl_half_period);
    }
    gpio_set_level(sda_io, 0);  // setup for STOP
    gpio_set_level(scl_io, 1);
    esp_rom_delay_us(scl_half_period);
    gpio_set_level(sda_io, 1);  // STOP, SDA low -> high while SCL is HIGH
    i2c_conf_pins(i2c_num, sda_io, scl_io, 1, 1);
#else
    i2c_hal_master_clr_bus(&i2c_ctx.hal);
#endif
    return ESP_OK;
}

/**if the power and SDA/SCL wires are in proper condition, everything works find with reading the
 *slave. If we remove the power supply for the slave during I2C is reading, or directly connect SDA
 *or SCL to ground, this would cause the I2C FSM get stuck in wrong state, all we can do is to reset
 *the I2C hardware in this case.
 **/
esp_err_t mini_i2c_hw_fsm_reset() {
    i2c_port_t i2c_num = 0;
#if !SOC_I2C_SUPPORT_HW_FSM_RST
    int scl_low_period, scl_high_period;
    int scl_start_hold, scl_rstart_setup;
    int scl_stop_hold, scl_stop_setup;
    int sda_hold, sda_sample;
    int timeout;
    uint8_t filter_cfg;

    i2c_hal_get_scl_timing(&i2c_ctx.hal, &scl_high_period, &scl_low_period);
    i2c_hal_get_start_timing(&i2c_ctx.hal, &scl_rstart_setup, &scl_start_hold);
    i2c_hal_get_stop_timing(&i2c_ctx.hal, &scl_stop_setup, &scl_stop_hold);
    i2c_hal_get_sda_timing(&i2c_ctx.hal, &sda_sample, &sda_hold);
    i2c_hal_get_tout(&i2c_ctx.hal, &timeout);
    i2c_hal_get_filter(&i2c_ctx.hal, &filter_cfg);

    // to reset the I2C hw module, we need re-enable the hw
    periph_module_disable(i2c_periph_signal[0].module);
    mini_i2c_master_clear_bus(i2c_num);
    periph_module_enable(i2c_periph_signal[0].module);

    i2c_hal_master_init(&i2c_ctx.hal, i2c_num);
    i2c_hal_disable_intr_mask(&i2c_ctx.hal, I2C_LL_INTR_MASK);
    i2c_hal_clr_intsts_mask(&i2c_ctx.hal, I2C_LL_INTR_MASK);
    i2c_hal_set_scl_timing(&i2c_ctx.hal, scl_high_period, scl_low_period);
    i2c_hal_set_start_timing(&i2c_ctx.hal, scl_rstart_setup, scl_start_hold);
    i2c_hal_set_stop_timing(&i2c_ctx.hal, scl_stop_setup, scl_stop_hold);
    i2c_hal_set_sda_timing(&i2c_ctx.hal, sda_sample, sda_hold);
    i2c_hal_set_tout(&i2c_ctx.hal, timeout);
    i2c_hal_set_filter(&i2c_ctx.hal, filter_cfg);
#else
    i2c_hal_master_fsm_rst(&i2c_ctx.hal);
    mini_i2c_master_clear_bus(i2c_num);
#endif
    return ESP_OK;
}
