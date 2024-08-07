#pragma once

#include "fodom/model.h"

namespace fodom {

class Odom {
 public:
  template <typename Scalar_>
  void process_IMU(const IMUMeasurement<Scalar_>& meas);

 protected:
  IMUOutput::StateType X_;
};
}  // namespace fodom