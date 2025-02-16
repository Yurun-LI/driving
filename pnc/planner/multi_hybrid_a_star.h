#pragma once

#include "common/math/utils.h"
#include "common/math/vec.h"

namespace GraphSearch {
struct Pose {
  enum Gear {
    kForward = 1,
    kBackward = -1,
  };
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  Gear gear = kForward;

  Pose(double x, double y, double theta, const Gear& gear = kForward)
      : x(x), y(y), heading(theta), gear(gear) {}
  Pose(const math::Vec2d& position, double theta, Gear gear = kForward)
      : x(position.x), y(position.y), heading(theta), gear(gear) {}

  bool IsSamePosition(const Pose& other) const {
    return (std::abs(x - other.x) < math::kEpsilon &&
            std::abs(y - other.y) < math::kEpsilon);
  }
  bool operator==(const Pose& other) const {
    return (std::abs(x - other.x) < math::kEpsilon) &&
           (std::abs(y - other.y) < math::kEpsilon) &&
           (std::abs(math::NormalizeAngle(heading - other.heading)) <
            math::kEpsilon) &&
           (gear == other.gear);
  }
  struct PoseHash {
    std::size_t operator()(const Pose& pose) const {
      return std::hash<double>()(pose.x) ^ std::hash<double>()(pose.y) ^
             std::hash<double>()(pose.heading) ^
             std::hash<int>()(static_cast<int>(pose.gear));
    }
  };
};
class State {
 public:
  Pose pose;
  double g = math::kNan;
  double h = math::kINF;
  explicit State(const math::Vec2d& position, double theta,
                 Pose::Gear gear = Pose::kForward, double g = math::kNan,
                 double h = math::kINF)
      : pose(position, theta, gear), g(g), h(h) {}

  double cost() const { return g + h; }
};

}  // namespace GraphSearch