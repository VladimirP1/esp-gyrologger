class DynamicNotch {
   public:
    using ft = quat::base_type;
    DynamicNotch(double fs) {
        double dt = 1.0 / fs;
        double w = 400.0;
        double Q = 15.0;
        double w0 = 2 * M_PI * w * dt;
        double gb = 1.0 / sqrt(2);
        double beta = (sqrt(1.0 - gb * gb) / gb) * tan(w0 / Q / 2.0);
        double gain = 1.0 / (1.0 + beta);
        double cs = cos(w0);
        this->a_max = ft{cos(2 * M_PI * 150 * dt)};
        this->a_min = ft{cos(2 * M_PI * 800 * dt)};
        this->da_max = ft{(a_max - a_min) / 1000};
        this->cs = ft{cs};
        this->gain = ft{gain};
    }

    ft apply(ft inp) {
        ft b0 = gain, b1 = ft{-2.0} * cs * gain, b2 = b0, a1 = b1, a2 = ft{2.0} * gain - ft{1.0};
        ft out = ft{inp * b0 + x[1] * b1 + x[0] * b2 - y[1] * a1 - y[0] * a2};

        ft update0 = gain * (x[1] - y[1]) * 300 * out * 300 * -4;
        const ft k {.02};
        update = update + k * (update0 - update);
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
    ft x[2] = {{},{}}, y[2] = {{},{}};
    ft a_min{}, a_max{}, da_max{};
    ft lr{}, update{};
};