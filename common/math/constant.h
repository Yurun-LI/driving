#pragma once
#include <limits>

namespace math {
constexpr double kINF = std::numeric_limits<double>::infinity();
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kEpsilon = 1e-9;
}  // namespace math