#include "fodom/odom.h"

#include <iostream>

namespace fodom {

void Odom::propagate(const InputType& u, const Scalar dt) {
  // update the state
  X_ = IMUOutput::fwd(X_, u, dt);

  // propagate the covariance
}

void Odom::process_IMU(const IMUMeasurement<double>& meas) {
  std::cout << std::fixed << meas.stamp << "\n";
}
}  // namespace fodom