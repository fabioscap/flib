#pragma once

#include "fodom/math.h"

namespace fodom {
using Scalar = double;

template <typename Scalar_>
struct IMUMeasurement {
  Scalar_ stamp;
  Vector3_<Scalar_> ang_vel;
  Vector3_<Scalar_> lin_acc;
};

}  // namespace fodom