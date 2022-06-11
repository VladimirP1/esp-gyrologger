#pragma once
#include <functional>

namespace quat {
struct quat {
    quat(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
    double w{}, x{}, y{}, z{};
};
struct vec {
    vec(double x, double y, double z) : x(x), y(y), z(z) {}
    double x{}, y{}, z{};
    vec operator-(const vec &b) const { return {x - b.x, y - b.y, z - b.z}; }
    vec &operator+=(const vec &b) {
        x += b.x;
        y += b.y;
        z += b.z;
        return *this;
    }
};

quat from_aa(const vec &aa);
vec to_aa(const quat &q);
quat prod(const quat &p, const quat &q);
quat conj(const quat &q);
quat normalize(const quat &q);
vec rotate_point(const quat &q, const vec &p);
quat slerp(const quat &p, const quat &q, double t);
vec map(const vec &x, std::function<double(double)> f);
double norm(const vec &x);
};  // namespace quat