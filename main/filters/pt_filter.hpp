#pragma once

#include "compression/lib/fixquat.hpp"

class PtFilter {
   public:
    using ft = quat::base_type;
    PtFilter(int order, double cutoff, double fs) {
        this->order = order;
        double dT = 1.0 / fs;
        switch (order) {
            case 1: {
                double RC = 1 / (2 * M_PI * cutoff);
                gain = ft{dT / (RC + dT)};
            } break;
            case 2: {
                const double order = 2.0f;
                const double orderCutoffCorrection = 1 / sqrt(pow(2, 1.0 / order) - 1);
                double RC = 1 / (2 * orderCutoffCorrection * M_PI * cutoff);
                gain = ft{dT / (RC + dT)};
            } break;
            case 3: {
                const double order = 3.0f;
                const double orderCutoffCorrection = 1 / sqrt(pow(2, 1.0f / order) - 1);
                double RC = 1 / (2 * orderCutoffCorrection * M_PI * cutoff);
                gain = ft{dT / (RC + dT)};
            } break;
        }
    }

    ft apply(ft inp) {
        state[0] += gain * (inp - state[0]);
        if (order == 1) return state[0];
        state[1] += gain * (state[0] - state[1]);
        if (order == 2) return state[1];
        state[2] += gain * (state[1] - state[2]);
        return state[2];
    }

   private:
    int order{};
    ft gain{};
    ft state[3] = {};
};