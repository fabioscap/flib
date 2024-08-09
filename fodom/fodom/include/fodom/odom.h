#pragma once

#include "fodom/model.h"

namespace fodom {

class Odom {
 public:
  void process_IMU(const IMUMeasurement<double>& meas);

 protected:
  IMUOutput::StateType X_;
};
}  // namespace fodom