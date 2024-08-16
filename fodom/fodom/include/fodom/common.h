#pragma once

#include "fodom/math.h"

namespace fodom {
using Scalar = double;

template <typename Scalar_>
struct IMUMeasurement {
  Scalar_ stamp;              // s
  Vector3_<Scalar_> ang_vel;  // rad/s
  Vector3_<Scalar_> lin_acc;  // m/s^2
};

}  // namespace fodom