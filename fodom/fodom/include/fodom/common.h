#pragma once

#include "flib/math.h"

namespace fodom {
using Scalar = double;
using namespace flib::math;

template <typename Scalar_>
struct IMUMeasurement {
  Scalar_ stamp;              // s
  Vector3_<Scalar_> ang_vel;  // rad/s
  Vector3_<Scalar_> lin_acc;  // m/s^2
};

template <typename Scalar_>
struct Point {
  Point(Scalar x, Scalar y, Scalar z, Scalar t) : xyz{x, y, z}, stamp{t} {}
  Vector3_<Scalar_> xyz;  // m
  Scalar stamp;           // s
};

}  // namespace fodom