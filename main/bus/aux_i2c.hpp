#pragma once

struct aux_i2c_msg_t {
    char buf[32];
    int len;
};

void proc_aux_i2c();
void aux_i2c_send_blocking(aux_i2c_msg_t*msg);