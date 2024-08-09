#include "fodom/odom.h"

#include <iostream>

namespace fodom {

void Odom::process_IMU(const IMUMeasurement<double>& meas) {
  std::cout << std::fixed << meas.stamp << "\n";
}
}  // namespace fodom