#include "utils.h"
#include "constant.h"

double math::NormalizeAngle(double angle) {
  angle -= 2 * math::PI;
  while (angle < -math::PI)
    angle += 2 * math::PI;
  angle += 2 * math::PI;
  return angle;
}