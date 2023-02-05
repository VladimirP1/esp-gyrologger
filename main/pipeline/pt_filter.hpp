#pragma once
#include <math.h>
#include <cstdint>

struct PtFilter {
    static constexpr int32_t SHIFT = 27;
    PtFilter(int order, double cutoff, double fs) : order(order) {
        double dT = 1.0 / fs;
        switch (order) {
            case 1: {
                double RC = 1 / (2 * M_PI * cutoff);
                gain = dT / (RC + dT) * (1 << SHIFT);
            } break;
            case 2: {
                const double order = 2.0f;
                const double orderCutoffCorrection = 1 / sqrt(pow(2, 1.0 / order) - 1);
                double RC = 1 / (2 * orderCutoffCorrection * M_PI * cutoff);
                gain = dT / (RC + dT) * (1 << SHIFT);
            } break;
            case 3: {
                const double order = 3.0f;
                const double orderCutoffCorrection = 1 / sqrt(pow(2, 1.0f / order) - 1);
                double RC = 1 / (2 * orderCutoffCorrection * M_PI * cutoff);
                gain = dT / (RC + dT) * (1 << SHIFT);
            } break;
        }
    }
    void apply1(int16_t* s, int i);
    void apply3(int16_t* s);
   private:
    int order{};
    int64_t gain{};
    int32_t state[9] = {};
};