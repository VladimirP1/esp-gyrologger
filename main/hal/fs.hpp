#pragma once

struct FsSettings {
    bool external_sd{false};
    int pin_mosi{-1};
    int pin_miso{-1};
    int pin_clk{-1};
    int pin_cs{-1};
};

static constexpr char *mount_point = "/flash";

bool fs_init(FsSettings*);
void fs_deinit();
