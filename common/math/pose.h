#pragma once
#include "utils.h"
#include "vec.h"

namespace math {

class Pose : public Vec<double> {
 public:
  double theta;

  Pose() : Vec<double>(), theta(0) {}

  Pose(double x, double y, double theta) : Vec<double>(x, y), theta(theta) {}

  Pose(const Pose& p) = default;

  bool operator==(const Pose& p) const {
    return std::abs(x - p.x) < math::kEpsilon &&
           std::abs(y - p.y) < math::kEpsilon &&
           std::abs(math::NormalizeAngle(theta - p.theta)) < math::kEpsilon;
  }

  Pose operator+(const Pose& p) const = delete;
  Pose operator-(const Pose& p) const = delete;
  Pose operator*(double scalar) const = delete;
  Pose operator/(double scalar) const = delete;

  friend std::ostream& operator<<(std::ostream& os, const Pose& p) {
    os << "Pose(" << p.x << ", " << p.y << ", " << p.theta << ")";
    return os;
  }
};

}  // namespace math
