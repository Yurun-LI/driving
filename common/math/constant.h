// Author: Yurun-LI
// Date: 2025-05-31
// Description: Math constants for geometry and numerical computation.

#pragma once
#include <limits>

namespace math {
constexpr double kINF = std::numeric_limits<double>::infinity();
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
const double PI = 3.14159265358979323846;
const double PI_2 = 1.57079632679489661923;
const double PI_4 = 0.78539816339744830962;
const double PI_3 = 1.04719755119659774615;
const double PI_6 = 0.52359877559829887307;

}  // namespace math