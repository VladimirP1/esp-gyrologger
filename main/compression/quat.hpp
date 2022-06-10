#pragma once
#include <valarray>
#include <functional>

namespace quat {
struct quat {
    double w{}, x{}, y{}, z{};
};
struct vec {
    double x{},y{},z{};
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