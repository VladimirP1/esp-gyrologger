// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

#include <esp_check.h>
#include <utility>

esp_err_t storage_fat_init();
esp_err_t storage_fat_deinit();
std::pair<int, int> get_free_space_kb();

extern "C"{
    void wdt_off();
}