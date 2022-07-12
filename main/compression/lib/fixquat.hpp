// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

#include "fpm/fixed.hpp"
#include "fpm/math.hpp"

#include <stdint.h>

namespace quat {

using base_type = fpm::fixed<std::int32_t, std::int64_t, 27>;

struct vec {
    vec() : x(0), y(0), z(0) {}
    vec(base_type x, base_type y, base_type z) : x(x), y(y), z(z) {}
    base_type x{}, y{}, z{};
    inline vec operator-(const vec &b) const;
    inline vec &operator-=(const vec &b);
    inline vec operator+(const vec &b) const;
    inline vec &operator+=(const vec &b);
    inline vec operator/(base_type div) const;
    inline vec &operator/=(base_type div);
    inline vec operator*(base_type mul) const;
    inline vec &operator*=(base_type mul);
    inline base_type norm() const;
    inline vec normalized() const;
};

inline vec vec::operator-(const vec &b) const {
    vec ret = *this;
    ret -= b;
    return ret;
}

inline vec &vec::operator-=(const vec &b) {
    x -= b.x;
    y -= b.y;
    z -= b.z;
    return *this;
}

inline vec vec::operator+(const vec &b) const {
    vec ret = *this;
    ret += b;
    return ret;
}

inline vec &vec::operator+=(const vec &b) {
    x += b.x;
    y += b.y;
    z += b.z;
    return *this;
}

inline vec vec::operator/(base_type div) const {
    vec ret = *this;
    ret /= div;
    return ret;
}

inline vec &vec::operator/=(base_type div) {
    x /= div;
    y /= div;
    z /= div;
    return *this;
}

inline vec vec::operator*(base_type mul) const {
    vec ret = *this;
    ret *= mul;
    return ret;
}

inline vec &vec::operator*=(base_type mul) {
    x *= mul;
    y *= mul;
    z *= mul;
    return *this;
}

inline base_type vec::norm() const { return fpm::sqrt(x * x + y * y + z * z); }

inline vec vec::normalized() const {
    auto n = norm();
    if (n == base_type{}) {
        return {};
    }
    return (*this) / norm();
}

struct quat {
    inline quat();
    inline quat(base_type w, base_type x, base_type y, base_type z);
    explicit inline quat(const vec &aa);
    inline quat(const quat &q);

    inline quat &operator+=(const quat &b);
    inline quat operator+(const quat &b) const;
    inline quat &operator-=(const quat &b);
    inline quat operator-(const quat &b) const;
    inline quat &operator*=(const quat &b);
    inline quat operator*(const quat &b) const;
    inline quat operator*(base_type b) const;

    inline quat conj() const;
    inline base_type norm() const;
    inline quat normalized() const;
    inline vec rotate_point(const vec &p) const;
    inline quat slerp(const quat &qq, base_type t) const;
    inline vec axis_angle() const;

    base_type w{1}, x{}, y{}, z{};
};

inline quat::quat() : w((int16_t)1), x(0), y(0), z(0) {}

inline quat::quat(base_type w, base_type x, base_type y, base_type z) : w(w), x(x), y(y), z(z) {}

inline quat::quat(const vec &aa) {
    const base_type theta_squared = aa.x * aa.x + aa.y * aa.y + aa.z * aa.z;
    if (theta_squared > base_type::from_raw_value(16)) {
        const base_type theta = fpm::sqrt(theta_squared);
        const base_type half_theta = theta * base_type{0.5};
        const base_type k = fpm::sin(half_theta) / theta;
        (*this) = {fpm::cos(half_theta), aa.x * k, aa.y * k, aa.z * k};
    } else {
        const base_type k(0.5);
        *this = {base_type{1.}, aa.x * k, aa.y * k, aa.z * k};
    }
}

inline quat::quat(const quat &q) : w(q.w), x(q.x), y(q.y), z(q.z) {}

inline vec quat::axis_angle() const {
    const base_type sin_squared_theta = x * x + y * y + z * z;

    if (sin_squared_theta <= base_type{0.}) return {x * 2, y * 2, z * 2};

    const base_type sin_theta = fpm::sqrt(sin_squared_theta);
    const base_type &cos_theta = w;
    const base_type two_theta =
        base_type{2} * ((cos_theta < base_type{0}) ? fpm::atan2(-sin_theta, -cos_theta)
                                                   : fpm::atan2(sin_theta, cos_theta));
    const base_type k = two_theta / sin_theta;
    return {x * k, y * k, z * k};
}

inline quat &quat::operator+=(const quat &q) {
    w += q.w;
    x += q.x;
    y += q.y;
    z += q.z;
    return *this;
}

inline quat quat::operator+(const quat &q) const {
    quat ret = *this;
    ret += q;
    return ret;
}

inline quat &quat::operator-=(const quat &q) {
    w -= q.w;
    x -= q.x;
    y -= q.y;
    z -= q.z;
    return *this;
}

inline quat quat::operator-(const quat &q) const {
    quat ret = *this;
    ret -= q;
    return ret;
}

inline quat &quat::operator*=(const quat &q) {
    *this = {w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + x * q.w + y * q.z - z * q.y,
             w * q.y - x * q.z + y * q.w + z * q.x, w * q.z + x * q.y - y * q.x + z * q.w};
    return *this;
}

inline quat quat::operator*(const quat &q) const {
    quat ret = *this;
    ret *= q;
    return ret;
}

inline quat quat::operator*(base_type b) const { return {w * b, x * b, y * b, z * b}; }

inline quat quat::conj() const {
    const quat &q = *this;
    return {w, -x, -y, -z};
}

inline quat quat::normalized() const {
    base_type n = norm();
    if (n == base_type{}) {
        return {base_type{}, base_type{}, base_type{}, base_type{}};
    }
    return {w / n, x / n, y / n, z / n};
}

inline quat normalize(const quat &q) {
    base_type norm = fpm::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

inline vec quat::rotate_point(const vec &p) const {
    auto qq = (*this) * quat{base_type{0}, p.x, p.y, p.z} * (*this).conj();
    return {qq.x, qq.y, qq.z};
}

inline quat quat::slerp(const quat &qq, base_type t) const {
    quat q = qq;
    base_type pq_dot = w * q.w + x * q.x + y * q.y + z * q.z;
    if (pq_dot < base_type{0}) {
        q = {q.w * -1, q.x * -1, q.y * -1, q.z * -1};
        pq_dot = pq_dot * -1;
    }

    base_type mult1, mult2;
    const base_type theta = fpm::acos(pq_dot);

    // TODO: check if differentiable
    if (theta > base_type{1e-9}) {
        const base_type sin_theta = fpm::sin(theta);
        mult1 = fpm::sin((base_type{1} - t) * theta) / sin_theta;
        mult2 = fpm::sin(t * theta) / sin_theta;
    } else {
        mult1 = base_type{(int16_t)1} - t;
        mult2 = t;
    }

    return {mult1 * w + mult2 * q.w, mult1 * x + mult2 * q.x, mult1 * y + mult2 * q.y,
            mult1 * z + mult2 * q.z};
}

inline base_type quat::norm() const { return fpm::sqrt(w * w + x * x + y * y + z * z); }

}  // namespace quat