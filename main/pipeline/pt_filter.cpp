#include "pt_filter.hpp"

extern "C" {
#include "esp_attr.h"
}

void IRAM_ATTR PtFilter::apply1(int16_t* s, int i) {
    int32_t* _state = state + 3 * i;
    _state[0] += (gain * ((int)*s * 4 - _state[0])) >> SHIFT;
    if (order == 1) {
        *s = _state[0] / 4;
        return;
    }
    _state[1] += (gain * (_state[0] - _state[1])) >> SHIFT;
    if (order == 2) {
        *s = _state[1] / 4;
        return;
    }
    _state[2] += (gain * (_state[1] - _state[2])) >> SHIFT;
    *s = _state[2] / 4;
}

void IRAM_ATTR PtFilter::apply3(int16_t* s) {
    apply1(s + 0, 0);
    apply1(s + 1, 1);
    apply1(s + 2, 2);
}