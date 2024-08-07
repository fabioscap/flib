#include "fodom/odom.h"

#include <iostream>

namespace fodom {
template <typename Scalar_>
void Odom::process_IMU(const IMUMeasurement<Scalar_>& meas) {
  std::cout << "Ciaooo!!" << "\n";
}
}  // namespace fodom