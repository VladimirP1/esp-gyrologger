#include <cmath>
#include <iostream>

constexpr std::initializer_list<double> kVarianceTable = {
    0.015625, 0.03125, 0.0625, 0.125, 0.25, 0.5,   1.0,   2.0,
    4.0,      8.0,     16.0,   32.0,  64.0, 128.0, 256.0, 512.0};

class LaplaceModel {
   public:
    explicit LaplaceModel(int i_var, int scale) : i_var_(i_var), scale_(scale) {}

    double b() const { return sqrt(var() / 2); }

    double var() const { return *(kVarianceTable.begin() + i_var_); }

    int cdf(int x) const {
        if (x <= -128) return 0;
        if (x > 128) return 1 << scale_;

        double cum{};
        double xs{x - .5};
        if (xs < 0) {
            cum = exp(xs / b()) / 2;
        } else {
            cum = 1 - exp(-xs / b()) / 2;
        }

        return int(cum * ((1 << scale_) - 257)) + (x + 128);
    }

    int icdf(int y) const {
        int l{-129}, r{129};
        while (l + 1 != r) {
            int mid = (l + r) / 2;
            if (cdf(mid) <= y && cdf(mid + 1) > y) {
                return mid;
            }
            if (cdf(mid) <= y) {
                l = mid;
            } else {
                r = mid;
            }
        }
        return r;
    }

    int freq(int x) const { return cdf(x + 1) - cdf(x); }

   private:
    int i_var_, scale_;
};

int main() {
    for (int i = 0; i < 16; ++i) {
        LaplaceModel m(i, 15);
        std::cout << "const uint16_t cdf_" << i << "[] = {";
        for (int i = -128; i <= 129; ++i) {
            std::cout << m.cdf(i) << ", ";
        }
        std::cout << "};" << std::endl;
    }
    std::cout << std::endl;
    
    std::cout << "const uint16_t* cdf_ptrs[] = {";
    for (int i = 0; i < 16; ++i) {
        std::cout << "cdf_" << i << ", ";
    }
    std::cout << "};" << std::endl;

    return 0;
}