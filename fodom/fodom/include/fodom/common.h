#pragma once

#include "flib/common.h"
#include "flib/math.h"

namespace fodom {
using Scalar = flib::Scalar;
using namespace flib::math;

template <typename Scalar_>
struct IMUMeasurement {
  Scalar_ stamp;              // s
  Vector3_<Scalar_> ang_vel;  // rad/s
  Vector3_<Scalar_> lin_acc;  // m/s^2
};

}  // namespace fodom