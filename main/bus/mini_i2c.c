#include "mini_i2c.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_private/periph_ctrl.h"
#include "esp_clk_tree.h"
#include "clk_ctrl_os.h"
#include "hal/gpio_hal.h"
#include "hal/i2c_ll.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "i2c"

#if !SOC_RCC_IS_INDEPENDENT
#define I2C_RCC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define I2C_RCC_ATOMIC()
#endif

#if SOC_PERIPH_CLK_CTRL_SHARED
#define I2C_CLOCK_SRC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define I2C_CLOCK_SRC_ATOMIC()
#endif

#ifndef GPIO_OUTPUT_SET
#define GPIO_OUTPUT_SET gpio_output_set
#endif

static void IRAM_ATTR i2c_isr_handler(void *arg) {
    struct i2c_ctx *ctx = (struct i2c_ctx *)arg;
    i2c_dev_t *dev = ctx->dev;
    uint32_t int_mask;
    i2c_ll_get_intr_mask(dev, &int_mask);
    i2c_ll_clear_intr_mask(dev, int_mask);
    if (int_mask == 0) {
        return;
    }

    if (int_mask == I2C_LL_INTR_NACK) {
        ctx->status = I2C_STATUS_NACK;
    } else if (int_mask == I2C_LL_INTR_TIMEOUT) {
        ctx->status = I2C_STATUS_TIMEOUT;
    } else if (int_mask == I2C_LL_INTR_ARBITRATION) {
        ctx->status = I2C_STATUS_ARB_LOST;
    } else if (int_mask == I2C_LL_INTR_MST_COMPLETE) {
        ctx->status = I2C_STATUS_IDLE;
    } else if (int_mask == I2C_LL_INTR_END_DETECT) {
        ctx->status = I2C_STATUS_IDLE;
    } else {
        return;
    }

    if (ctx->callback) {
        void (*callback)(void *args);
        callback = ctx->callback;
        ctx->callback = NULL;
        callback(ctx->callback_args);
    }
}

static inline __attribute__((always_inline)) void mini_i2c_enable_hw(struct i2c_ctx *ctx) {
    I2C_RCC_ATOMIC() {
        i2c_ll_enable_bus_clock(ctx->i2c_port, true);
        i2c_ll_reset_register(ctx->i2c_port);
    }
}

static inline __attribute__((always_inline)) void mini_i2c_disable_hw(struct i2c_ctx *ctx) {
    I2C_RCC_ATOMIC() {
        i2c_ll_enable_bus_clock(ctx->i2c_port, false);
        i2c_ll_reset_register(ctx->i2c_port);
    }
}

static IRAM_ATTR esp_err_t mini_i2c_clear_bus(struct i2c_ctx *ctx) {
#if !SOC_I2C_SUPPORT_HW_CLR_BUS
    const int scl_half_period = 5;  // use standard 100kHz data rate
    int i = 0;
    esp_rom_gpio_connect_out_signal(ctx->sda, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(ctx->scl, SIG_GPIO_OUT_IDX, false, false);
    // If a SLAVE device was in a read operation when the bus was interrupted, the SLAVE device is
    // controlling SDA. The only bit during the 9 clock cycles of a READ byte the MASTER(ESP32) is
    // guaranteed control over is during the ACK bit period. If the slave is sending a stream of
    // ZERO bytes, it will only release SDA during the ACK bit period. So, this reset code needs to
    // synchronous the bit stream with, Either, the ACK bit, Or a 1 bit to correctly generate a STOP
    // condition.
    gpio_ll_set_level(&GPIO, ctx->scl, 0);
    gpio_ll_set_level(&GPIO, ctx->sda, 1);
    esp_rom_delay_us(scl_half_period);

    while (!gpio_ll_get_level(&GPIO, ctx->sda) && (i++ < 9)) {
        gpio_ll_set_level(&GPIO, ctx->scl, 1);
        esp_rom_delay_us(scl_half_period);
        gpio_ll_set_level(&GPIO, ctx->scl, 0);
        esp_rom_delay_us(scl_half_period);
    }
    gpio_ll_set_level(&GPIO, ctx->sda, 0);
    gpio_ll_set_level(&GPIO, ctx->scl, 1);
    esp_rom_delay_us(scl_half_period);
    gpio_ll_set_level(&GPIO, ctx->sda, 1);  // STOP, SDA low -> high while SCL is HIGH

    esp_rom_gpio_connect_out_signal(ctx->sda, i2c_periph_signal[ctx->i2c_port].sda_out_sig, 0, 0);
    esp_rom_gpio_connect_out_signal(ctx->scl, i2c_periph_signal[ctx->i2c_port].scl_out_sig, 0, 0);
#else
    i2c_ll_master_clr_bus(ctx->dev, I2C_LL_RESET_SLV_SCL_PULSE_NUM_DEFAULT);
#endif
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_hw_fsm_reset(struct i2c_ctx *ctx) {
#if !SOC_I2C_SUPPORT_HW_FSM_RST
    uint32_t filter_cfg = ctx->dev->filter_cfg.val;
    uint32_t scl_low_per = ctx->dev->scl_low_period.val;
    uint32_t sda_hold = ctx->dev->sda_hold.val;
    uint32_t sda_sample = ctx->dev->sda_sample.val;
    uint32_t scl_high_per = ctx->dev->scl_high_period.val;
    uint32_t scl_start_hold = ctx->dev->scl_start_hold.val;
    uint32_t scl_rstart_setup = ctx->dev->scl_rstart_setup.val;
    uint32_t scl_stop_hold = ctx->dev->scl_stop_hold.val;
    uint32_t scl_stop_setup = ctx->dev->scl_stop_setup.val;
    uint32_t scl_st_timeout = ctx->dev->scl_st_time_out.val;
    uint32_t scl_main_st_timeout = ctx->dev->scl_main_st_time_out.val;

    mini_i2c_clear_bus(ctx);
    mini_i2c_disable_hw(ctx);
    mini_i2c_enable_hw(ctx);

    i2c_ll_master_init(ctx->dev);

    i2c_ll_disable_intr_mask(ctx->dev, I2C_LL_INTR_MASK);
    i2c_ll_clear_intr_mask(ctx->dev, I2C_LL_INTR_MASK);

    ctx->dev->scl_low_period.val = scl_low_per;
    ctx->dev->sda_hold.val = sda_hold;
    ctx->dev->sda_sample.val = sda_sample;
    ctx->dev->scl_high_period.val = scl_high_per;
    ctx->dev->scl_start_hold.val = scl_start_hold;
    ctx->dev->scl_rstart_setup.val = scl_rstart_setup;
    ctx->dev->scl_stop_hold.val = scl_stop_hold;
    ctx->dev->scl_stop_setup.val = scl_stop_setup;
    ctx->dev->scl_st_time_out.val = scl_st_timeout;
    ctx->dev->scl_main_st_time_out.val = scl_main_st_timeout;
    ctx->dev->filter_cfg.val = filter_cfg;

    i2c_ll_update(ctx->dev);
#else
    i2c_ll_master_fsm_rst(ctx->dev);
    minii2c_clear_bus(ctx);
#endif

    return ESP_OK;
}

esp_err_t mini_i2c_double_stop_timing(struct i2c_ctx *ctx) {
    int scl_stop_hold, scl_stop_setup;
    i2c_ll_get_stop_timing(ctx->dev, &scl_stop_setup, &scl_stop_hold);
    scl_stop_hold *= 2;
    i2c_ll_master_set_stop_timing(ctx->dev, scl_stop_setup, scl_stop_hold);
    i2c_ll_update(ctx->dev);
    return ESP_OK;
}

esp_err_t mini_i2c_init(struct i2c_ctx *ctx, int sda, int scl, uint32_t i2c_freq, int i2c_port) {
    memset(ctx, 0, sizeof(struct i2c_ctx));
    ctx->sda = sda;
    ctx->scl = scl;
    ctx->i2c_freq = i2c_freq;
    ctx->i2c_port = i2c_port;
    ctx->dev = I2C_LL_GET_HW(i2c_port);
    ctx->clk_src = I2C_CLK_SRC_DEFAULT;
    ctx->periph_src_clk_hz = 0;
    ctx->status = I2C_STATUS_IDLE;
    ctx->callback = NULL;
    ctx->callback_args = NULL;

    mini_i2c_enable_hw(ctx);
    I2C_CLOCK_SRC_ATOMIC() { i2c_ll_enable_controller_clock(ctx->dev, true); }

    gpio_config_t sda_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_down_en = false,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pin_bit_mask = 1ULL << ctx->sda,
        
    };
    if (gpio_set_level(ctx->sda, 1) != ESP_OK) {
        ESP_LOGE(TAG, "ctx->sda set level failed");
        return ESP_FAIL;
    }
    if (gpio_config(&sda_conf) != ESP_OK) {
        ESP_LOGE(TAG, "ctx->sda gpio config failed");
        return ESP_FAIL;
    }
    gpio_set_drive_capability(ctx->sda, GPIO_DRIVE_CAP_0);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[ctx->sda], PIN_FUNC_GPIO);
    esp_rom_gpio_connect_out_signal(ctx->sda, i2c_periph_signal[ctx->i2c_port].sda_out_sig, 0, 0);
    esp_rom_gpio_connect_in_signal(ctx->sda, i2c_periph_signal[ctx->i2c_port].sda_in_sig, 0);

    gpio_config_t scl_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_down_en = false,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pin_bit_mask = 1ULL << ctx->scl,
    };
    if (gpio_set_level(ctx->scl, 1) != ESP_OK) {
        ESP_LOGE(TAG, "ctx->scl set level failed");
        return ESP_FAIL;
    }
    if (gpio_config(&scl_conf) != ESP_OK) {
        ESP_LOGE(TAG, "ctx->scl gpio config failed");
        return ESP_FAIL;
    }
    gpio_set_drive_capability(ctx->scl, GPIO_DRIVE_CAP_0);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[ctx->scl], PIN_FUNC_GPIO);
    esp_rom_gpio_connect_out_signal(ctx->scl, i2c_periph_signal[ctx->i2c_port].scl_out_sig, 0, 0);
    esp_rom_gpio_connect_in_signal(ctx->scl, i2c_periph_signal[ctx->i2c_port].scl_in_sig, 0);

#if SOC_I2C_SUPPORT_RTC
    if (ctx->clk_src == I2C_CLK_SRC_RC_FAST) {
        // RC_FAST clock is not enabled automatically on start up, we enable it here manually.
        // Note there's a ref count in the enable/disable function, we must call them in pair in the
        // driver.
        periph_rtc_dig_clk8m_enable();
    }
#endif  // SOC_I2C_SUPPORT_RTC

    if (esp_clk_tree_src_get_freq_hz(ctx->clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX,
                                     &ctx->periph_src_clk_hz) != ESP_OK) {
        ESP_LOGE(TAG, "failed to get i2c clock frequency");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "bus clock source frequency: %" PRIu32 "hz", ctx->periph_src_clk_hz);

    i2c_ll_master_init(ctx->dev);
    // MSB
    i2c_ll_set_data_mode(ctx->dev, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
    // Reset fifo
    i2c_ll_txfifo_rst(ctx->dev);
    i2c_ll_rxfifo_rst(ctx->dev);

    I2C_CLOCK_SRC_ATOMIC() {
        i2c_ll_set_source_clk(ctx->dev, ctx->clk_src);
        i2c_hal_clk_config_t clk_cal = {0};
        i2c_ll_master_cal_bus_clk(ctx->periph_src_clk_hz, ctx->i2c_freq, &clk_cal);
        i2c_ll_master_set_bus_timing(ctx->dev, &clk_cal);
    }

    i2c_ll_master_set_fractional_divider(ctx->dev, 0, 0);
    i2c_ll_update(ctx->dev);

    i2c_ll_disable_intr_mask(ctx->dev, I2C_LL_INTR_MASK);
    i2c_ll_clear_intr_mask(ctx->dev, I2C_LL_INTR_MASK);
    if (esp_intr_alloc(i2c_periph_signal[ctx->i2c_port].irq, ESP_INTR_FLAG_IRAM, i2c_isr_handler,
                       (void *)ctx, &ctx->int_hndl) != ESP_OK) {
        return ESP_FAIL;
    }

    ctx->mtx = xSemaphoreCreateMutex();

    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_read_reg_sync(struct i2c_ctx *ctx, uint8_t dev_adr, uint8_t reg_adr,
                                           uint8_t *bytes, uint8_t n_bytes) {
    if (!xSemaphoreTakeFromISR(ctx->mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }
    i2c_dev_t *dev = ctx->dev;
    i2c_ll_rxfifo_rst(dev);
    i2c_ll_txfifo_rst(dev);

    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, (dev_adr << 1) | 1};
    i2c_ll_write_txfifo(dev, data, 3);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 2, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 1, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    if (n_bytes > 1) {
        i2c_ll_hw_cmd_t hw_cmd = {
            .op_code = I2C_LL_CMD_READ, .byte_num = n_bytes - 1, .ack_val = 0};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_READ, .byte_num = 1, .ack_val = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }

    ctx->status = I2C_STATUS_ACTIVE;
    i2c_ll_master_enable_tx_it(dev);
    i2c_ll_master_trans_start(dev);

    int cnt = 1000;
    while (cnt && ctx->status == I2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(ctx);
        --cnt;
    }

    // ESP_LOGI(TAG, "SR      %08x", (unsigned int)dev->sr.val);
    // ESP_LOGI(TAG, "INT_EN  %08x", (unsigned int)dev->int_ena.val);
    // ESP_LOGI(TAG, "INT_STS %08x", (unsigned int)dev->int_status.val);
    // ESP_LOGI(TAG, "txfifo_cnt: %d", (unsigned int)dev->sr.tx_fifo_cnt);
    // ESP_LOGI(TAG, "rxfifo_cnt: %d", (unsigned int)dev->sr.rx_fifo_cnt);

    #if defined(CONFIG_IDF_TARGET_ESP32C3)
    int act_n_bytes = dev->sr.rx_fifo_cnt;
    #elif defined(CONFIG_IDF_TARGET_ESP32S3)
    int act_n_bytes = dev->sr.rxfifo_cnt;
    #endif
    
    if (!cnt || act_n_bytes != n_bytes) {
        xSemaphoreGiveFromISR(ctx->mtx, NULL);
        return ESP_FAIL;
    }

    i2c_ll_read_rxfifo(dev, bytes, n_bytes);

    xSemaphoreGiveFromISR(ctx->mtx, NULL);

    if (ctx->status != I2C_STATUS_IDLE) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_read_reg_callback(struct i2c_ctx *ctx, uint8_t dev_adr,
                                               uint8_t reg_adr, uint8_t n_bytes,
                                               void (*callback)(void *args), void *callback_args) {
    if (!xSemaphoreTakeFromISR(ctx->mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }
    i2c_dev_t *dev = ctx->dev;
    i2c_ll_rxfifo_rst(dev);
    i2c_ll_txfifo_rst(dev);

    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, (dev_adr << 1) | 1};
    i2c_ll_write_txfifo(dev, data, 3);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 2, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 1, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    if (n_bytes > 1) {
        i2c_ll_hw_cmd_t hw_cmd = {
            .op_code = I2C_LL_CMD_READ, .byte_num = n_bytes - 1, .ack_val = 0};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_READ, .byte_num = 1, .ack_val = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }

    ctx->callback = callback;
    ctx->callback_args = callback_args;
    ctx->status = I2C_STATUS_ACTIVE;
    i2c_ll_master_enable_tx_it(dev);
    i2c_ll_master_trans_start(dev);

    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_read_reg_get_result(struct i2c_ctx *ctx, uint8_t *bytes,
                                                 uint8_t n_bytes) {
    i2c_ll_read_rxfifo(ctx->dev, bytes, n_bytes);

    xSemaphoreGiveFromISR(ctx->mtx, NULL);

    if (ctx->status != I2C_STATUS_IDLE) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_write_reg_sync(struct i2c_ctx *ctx, uint8_t dev_adr, uint8_t reg_adr,
                                            uint8_t byte) {
    if (!xSemaphoreTakeFromISR(ctx->mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }
    i2c_dev_t *dev = ctx->dev;
    i2c_ll_rxfifo_rst(dev);
    i2c_ll_txfifo_rst(dev);

    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, byte};
    i2c_ll_write_txfifo(dev, data, 3);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 3, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }

    ctx->status = I2C_STATUS_ACTIVE;
    i2c_ll_master_enable_tx_it(dev);
    i2c_ll_master_trans_start(dev);

    while (ctx->status == I2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(ctx);
    }

    xSemaphoreGiveFromISR(ctx->mtx, NULL);

    if (ctx->status != I2C_STATUS_IDLE) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_write_reg2_sync(struct i2c_ctx *ctx, uint8_t dev_adr, uint8_t reg_adr,
                                             uint8_t byte1, uint8_t byte2) {
    if (!xSemaphoreTakeFromISR(ctx->mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }
    i2c_dev_t *dev = ctx->dev;
    i2c_ll_rxfifo_rst(dev);
    i2c_ll_txfifo_rst(dev);

    int idx = 0;
    uint8_t data[] = {dev_adr << 1, reg_adr, byte1, byte2};
    i2c_ll_write_txfifo(dev, data, 4);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = 4, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }

    ctx->status = I2C_STATUS_ACTIVE;
    i2c_ll_master_enable_tx_it(dev);
    i2c_ll_master_trans_start(dev);

    while (ctx->status == I2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(ctx);
    }

    xSemaphoreGiveFromISR(ctx->mtx, NULL);

    if (ctx->status != I2C_STATUS_IDLE) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t IRAM_ATTR mini_i2c_write_n_sync(struct i2c_ctx *ctx, uint8_t *data, int len) {
    if (!xSemaphoreTakeFromISR(ctx->mtx, NULL)) {
        return ESP_ERR_NOT_FINISHED;
    }
    i2c_dev_t *dev = ctx->dev;
    i2c_ll_rxfifo_rst(dev);
    i2c_ll_txfifo_rst(dev);

    int idx = 0;
    i2c_ll_write_txfifo(dev, data, len);
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_RESTART};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_WRITE, .byte_num = len, .ack_en = 1};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }
    {
        i2c_ll_hw_cmd_t hw_cmd = {.op_code = I2C_LL_CMD_STOP};
        i2c_ll_master_write_cmd_reg(dev, hw_cmd, idx++);
    }

    ctx->status = I2C_STATUS_ACTIVE;
    i2c_ll_master_enable_tx_it(dev);
    i2c_ll_master_trans_start(dev);

    while (ctx->status == I2C_STATUS_ACTIVE) {
        esp_rom_delay_us(10);
        i2c_isr_handler(ctx);
    }

    xSemaphoreGiveFromISR(ctx->mtx, NULL);

    if (ctx->status != I2C_STATUS_IDLE) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mini_i2c_set_timing(struct i2c_ctx *ctx, uint32_t i2c_freq) {
    ctx->i2c_freq = i2c_freq;
    I2C_CLOCK_SRC_ATOMIC() {
        i2c_ll_set_source_clk(ctx->dev, ctx->clk_src);
        i2c_hal_clk_config_t clk_cal = {0};
        i2c_ll_master_cal_bus_clk(ctx->periph_src_clk_hz, ctx->i2c_freq, &clk_cal);
        i2c_ll_master_set_bus_timing(ctx->dev, &clk_cal);
    }

    i2c_ll_master_set_fractional_divider(ctx->dev, 0, 0);
    i2c_ll_update(ctx->dev);
    return ESP_OK;
}