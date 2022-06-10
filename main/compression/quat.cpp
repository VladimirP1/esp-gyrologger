#include "quat.hpp"

#include <stdint.h>
#include <valarray>

namespace quat {
quat from_aa(const vec &aa) {
    const double theta_squared = aa.x * aa.x + aa.y * aa.y + aa.z * aa.z;
    if (theta_squared > 0.) {
        const double theta = sqrt(theta_squared);
        const double half_theta = theta * 0.5;
        const double k = sin(half_theta) / theta;
        return {cos(half_theta), aa[0] * k, aa[1] * k, aa[2] * k};
    } else {
        const double k(0.5);
        return {1., aa[0] * k, aa[1] * k, aa[2] * k};
    }
}

vec to_aa(const quat &q) {
    const vec &xyz = q[std::slice(1, 3, 1)];
    const double sin_squared_theta = (xyz * xyz).sum();

    if (sin_squared_theta <= 0.) return map(xyz, [](double x) { return x * 2; });

    const double sin_theta = sqrt(sin_squared_theta);
    const double &cos_theta = q[0];
    const double two_theta =
        2. * ((cos_theta < 0.) ? atan2(-sin_theta, -cos_theta) : atan2(sin_theta, cos_theta));
    const double k = two_theta / sin_theta;
    return xyz * k;
}

quat prod(const quat &p, const quat &q) {
    return {p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
            p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
            p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
            p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]};
}

quat conj(const quat &q) {
    quat qq = q;
    qq[std::slice(1, 3, 1)] = -vec(qq[std::slice(1, 3, 1)]);
    return qq;
}

quat normalize(const quat &q) { return q / sqrt((q * q).sum()); }

vec rotate_point(const quat &q, const quat &p) {
    return prod(q, prod({0, p[0], p[1], p[2]}, conj(q)))[std::slice(1, 3, 1)];
}

quat slerp(const quat &p, const quat &qq, double t) {
    quat q = qq;
    if ((p * q).sum() < 0) {
        q = -q;
    }

    double mult1, mult2;
    const double theta = acos((p * q).sum());

    // TODO: check if differentiable
    if (theta > 1e-9) {
        const double sin_theta = sin(theta);
        mult1 = sin((1 - t) * theta) / sin_theta;
        mult2 = sin(t * theta) / sin_theta;
    } else {
        mult1 = 1 - t;
        mult2 = t;
    }

    return mult1 * p + mult2 * q;
}

vec map(const vec &x, std::function<double(double)> f) {
    vec ret(x.size());
    for (size_t i = 0; i < x.size(); ++i) ret[i] = f(x[i]);
    return ret;
}

double norm(const vec &x) { return sqrt((x * x).sum()); }

}  // namespace quat