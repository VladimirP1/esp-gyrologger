#include "quat.hpp"

#include <stdint.h>
#include <math.h>

namespace quat {
quat from_aa(const vec &aa) {
    const double theta_squared = aa.x * aa.x + aa.y * aa.y + aa.z * aa.z;
    if (theta_squared > 0.) {
        const double theta = sqrt(theta_squared);
        const double half_theta = theta * 0.5;
        const double k = sin(half_theta) / theta;
        return {cos(half_theta), aa.x * k, aa.y * k, aa.z * k};
    } else {
        const double k(0.5);
        return {1., aa.x * k, aa.y * k, aa.z * k};
    }
}

vec to_aa(const quat &q) {
    const double sin_squared_theta = q.x * q.x + q.y * q.y + q.z * q.z;

    if (sin_squared_theta <= 0.) return {q.x * 2, q.y * 2, q.z * 2};

    const double sin_theta = sqrt(sin_squared_theta);
    const double &cos_theta = q.w;
    const double two_theta =
        2. * ((cos_theta < 0.) ? atan2(-sin_theta, -cos_theta) : atan2(sin_theta, cos_theta));
    const double k = two_theta / sin_theta;
    return {q.x * k, q.y * k, q.z * k};
}

quat prod(const quat &p, const quat &q) {
    return {p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
            p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
            p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x,
            p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w};
}

quat conj(const quat &q) { return {q.w, -q.x, -q.y, -q.z}; }

quat normalize(const quat &q) {
    double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

vec rotate_point(const quat &q, const vec &p) {
    auto qq = prod(q, prod({0, p.x, p.y, p.z}, conj(q)));
    return {qq.x, qq.y, qq.z};
}

quat slerp(const quat &p, const quat &qq, double t) {
    quat q = qq;
    double pq_dot = p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z;
    if (pq_dot < 0) {
        q = {-q.w, -q.x, -q.y, -q.z};
        pq_dot = -pq_dot;
    }

    double mult1, mult2;
    const double theta = acos(pq_dot);

    // TODO: check if differentiable
    if (theta > 1e-9) {
        const double sin_theta = sin(theta);
        mult1 = sin((1 - t) * theta) / sin_theta;
        mult2 = sin(t * theta) / sin_theta;
    } else {
        mult1 = 1 - t;
        mult2 = t;
    }

    return {mult1 * p.w + mult2 * q.w, mult1 * p.x + mult2 * q.x, mult1 * p.y + mult2 * q.y,
            mult1 * p.z + mult2 * q.z};
}

vec map(const vec &x, std::function<double(double)> f) {
    return {f(x.x), f(x.y), f(x.z)};
}

double norm(const vec &x) { return sqrt(x.x * x.x + x.y * x.y + x.z * x.z); }

}  // namespace quat