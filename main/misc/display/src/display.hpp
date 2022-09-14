#pragma once
#include <cstdint>

void display_setup();
void display_task(void* params);
void oled_capture(uint8_t *out);
int oled_get_width();
int oled_get_height();