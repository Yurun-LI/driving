#pragma once
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>

#include "constant.h"

namespace math {
template<typename T> class Vec {
public:
    T x, y;

    Vec()
        : x(0)
        , y(0) {}
    Vec(T x, T y)
        : x(x)
        , y(y) {}
    Vec(const Vec& v) = default;
    Vec(Vec&& v)      = default;
    Vec& operator=(Vec&& v) = default;
    bool operator==(const Vec& v) const { return std::abs(x - v.x) < math::kEpsilon && std::abs(y - v.y) < math::kEpsilon; }

    Vec operator+(const Vec& v) const { return Vec(x + v.x, y + v.y); }

    Vec operator-(const Vec& v) const { return Vec(x - v.x, y - v.y); }

    Vec operator*(T scalar) const { return Vec(x * scalar, y * scalar); }

    Vec operator/(T scalar) const {
        if (scalar == 0) {
            std::cerr << "Divide by zero error" << std::endl;
            return Vec(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        }
        return Vec(x / scalar, y / scalar);
    }

    T Dot(const Vec& v) const { return x * v.x + y * v.y; }
    T Cross(const Vec& v) const { return x * v.y - y * v.x; }
    T Rotate(double angle) const { return Vec(x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle)); }
    void Rotated(double angle) {
        x = x * cos(angle) - y * sin(angle);
        y = x * sin(angle) + y * cos(angle);
    }
    T Translate(const Vec& v) const { return Vec(x + v.x, y + v.y); }
    void Translated(const Vec& v) {
        x += v.x;
        y += v.y;
    }
    friend std::ostream& operator<<(std::ostream& os, const Vec& v) {
        os << "Vector(" << v.x << ", " << v.y << ")";
        return os;
    }
};
using Vec2d = Vec<double>;
using Vec2i = Vec<int>;
}   // namespace math