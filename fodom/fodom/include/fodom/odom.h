#pragma once

#include "fodom/model.h"

namespace fodom {

class Odom {
 public:
  using StateType = IMUOutput::StateType;
  using InputType = IMUOutput::InputType;
  using CovType = MatrixN_<Scalar, StateType::dim>;

  void process_IMU(const IMUMeasurement<Scalar>& meas);

  void propagate(const InputType& u, const Scalar dt);

  // protected:
  StateType X_;
  CovType S_ = 0 * CovType::Identity();

  Scalar b_gyro_cov_ = 0;  // (rad/s)^2
  Scalar b_accl_cov_ = 0;  // (m/s^2)^2

  Scalar n_gyro_cov_ = 0;  // (rad/s^2)^2
  Scalar n_accl_cov_ = 0;  // (m/s^3)^2
};
}  // namespace fodom