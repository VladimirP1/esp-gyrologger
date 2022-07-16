class DynamicNotch {
   public:
    using ft = quat::base_type;
    DynamicNotch(double fs, double min_freq, double max_freq, double Q, int lr, double lr_smooth) {
        double dt = 1.0 / fs;
        double w = (min_freq + max_freq) / 2;
        double w0 = 2 * M_PI * w * dt;
        double gb = 1.0 / sqrt(2);
        double beta = (sqrt(1.0 - gb * gb) / gb) * tan(w0 / Q / 2.0);
        double gain = 1.0 / (1.0 + beta);
        double cs = cos(w0);
        this->a_max = ft{cos(2 * M_PI * min_freq * dt)};
        this->a_min = ft{cos(2 * M_PI * max_freq * dt)};
        this->da_max = ft{(a_max - a_min) / 1000};
        this->cs = ft{cs};
        this->gain = ft{gain};
        this->lr_k = ft{lr_smooth};
    }

    ft apply(ft inp) {
        ft b0 = gain, b1 = ft{-2.0} * cs * gain, b2 = b0, a1 = b1, a2 = ft{2.0} * gain - ft{1.0};
        ft out = ft{inp * b0 + x[1] * b1 + x[0] * b2 - y[1] * a1 - y[0] * a2};

        ft update0 = gain * (x[1] - y[1]) * lr * out * lr * -4;
        update = update + lr_k * (update0 - update);
        if (update > da_max) update = da_max;
        if (update < -da_max) update = -da_max;
        cs -= update;
        cs = std::min(cs, a_max);
        cs = std::max(cs, a_min);

        x[0] = x[1];
        x[1] = inp;
        y[0] = y[1];
        y[1] = out;

        return out;
    }

   private:
    ft cs{}, gain{};
    ft x[2] = {{}, {}}, y[2] = {{}, {}};
    ft a_min{}, a_max{}, da_max{};
    ft lr_k{}, update{};
    int lr{};
};

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