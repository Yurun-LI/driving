// Author: Yurun-LI
// Date: 2024-05-31
// Description: 2D/2I vector math utility class with common vector operations.
//
#pragma once
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>

#include "constant.h"

namespace math {
template <typename T>
class Vec {
 public:
  T x, y;

  Vec() : x(0), y(0) {}

  Vec(T x, T y) : x(x), y(y) {}

  Vec(const Vec& v) = default;
  Vec(Vec&& v) = default;
  Vec& operator=(Vec&& v) = default;

  // Check if two vectors are equal (with epsilon tolerance)
  bool operator==(const Vec& v) const {
    return std::abs(x - v.x) < math::kEpsilon &&
           std::abs(y - v.y) < math::kEpsilon;
  }

  // Vector addition
  Vec operator+(const Vec& v) const { return Vec(x + v.x, y + v.y); }

  // Vector subtraction
  Vec operator-(const Vec& v) const { return Vec(x - v.x, y - v.y); }

  // Scalar multiplication
  Vec operator*(T scalar) const { return Vec(x * scalar, y * scalar); }

  // Scalar division (returns (inf, inf) if divided by zero)
  Vec operator/(T scalar) const {
    if (scalar == 0) {
      std::cerr << "Divide by zero error" << std::endl;
      return Vec(std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity());
    }
    return Vec(x / scalar, y / scalar);
  }

  // Dot product
  T Dot(const Vec& v) const { return x * v.x + y * v.y; }

  // 2D cross product
  T Cross(const Vec& v) const { return x * v.y - y * v.x; }

  // Rotate vector by angle (radians)
  Vec Rotate(double angle) const {
    return Vec(x * cos(angle) - y * sin(angle),
               x * sin(angle) + y * cos(angle));
  }

  // In-place rotation by angle (radians)
  void Rotated(double angle) {
    x = x * cos(angle) - y * sin(angle);
    y = x * sin(angle) + y * cos(angle);
  }

  // Translate by another vector
  Vec Translate(const Vec& v) const { return Vec(x + v.x, y + v.y); }

  // In-place translation by another vector
  void Translated(const Vec& v) {
    x += v.x;
    y += v.y;
  }

  // Euclidean length (magnitude)
  double Length() const { return std::sqrt(x * x + y * y); }

  // Squared length (no sqrt)
  double LengthSquared() const { return x * x + y * y; }

  // Return normalized vector (unit length)
  Vec Normalized() const {
    double len = Length();
    if (len < math::kEpsilon)
      return Vec(0, 0);
    return Vec(x / len, y / len);
  }

  // In-place normalization
  void Normalize() {
    double len = Length();
    if (len < math::kEpsilon) {
      x = 0;
      y = 0;
    } else {
      x /= len;
      y /= len;
    }
  }

  // Euclidean distance to another vector
  double DistanceTo(const Vec& v) const { return (*this - v).Length(); }

  // Angle (radians) from x-axis
  double Angle() const { return std::atan2(y, x); }

  // Signed angle (radians) to another vector
  double AngleTo(const Vec& v) const { return std::atan2(Cross(v), Dot(v)); }

  // Perpendicular vector (rotated 90 degrees CCW)
  Vec Perpendicular() const { return Vec(-y, x); }

  // Scalar projection of this vector onto v
  double ProjectOnto(const Vec& v) const {
    double len = v.Length();
    if (len < math::kEpsilon)
      return 0.0;
    return Dot(v) / len;
  }

  // Vector projection of this vector onto v
  Vec ProjectedOnto(const Vec& v) const {
    double len2 = v.LengthSquared();
    if (len2 < math::kEpsilon)
      return Vec(0, 0);
    double scale = Dot(v) / len2;
    return v * scale;
  }

  // Clamp vector length to max_length
  Vec Clamped(double max_length) const {
    double len = Length();
    if (len > max_length && len > math::kEpsilon) {
      return (*this) * (max_length / len);
    }
    return *this;
  }

  // Linear interpolation between this and v (ratio: 0 returns *this, 1 returns v)
  Vec Lerp(const Vec& v, double ratio) const {
    return (*this) * (1 - ratio) + v * ratio;
  }

  // Check if vector is (almost) zero
  bool IsZero() const {
    return std::abs(x) < math::kEpsilon && std::abs(y) < math::kEpsilon;
  }

  // Print vector to output stream
  friend std::ostream& operator<<(std::ostream& os, const Vec& v) {
    os << "Vector(" << v.x << ", " << v.y << ")";
    return os;
  }
};

using Vec2d = Vec<double>;
using Vec2i = Vec<int>;
}  // namespace math