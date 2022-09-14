#include "aux_i2c.hpp"
extern "C" {
#include "mini_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
}
#include "global_context.hpp"

void IRAM_ATTR proc_aux_i2c() {
    aux_i2c_msg_t msg;
    if (xQueueReceiveFromISR(gctx.aux_i2c_queue, &msg, nullptr)) {
        mini_i2c_write_n_sync((uint8_t*)msg.buf, msg.len);
    }
}

void aux_i2c_send_blocking(aux_i2c_msg_t* msg) {
    xQueueSend(gctx.aux_i2c_queue, msg, portMAX_DELAY);
}