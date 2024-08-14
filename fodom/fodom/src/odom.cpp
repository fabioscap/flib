#include "fodom/odom.h"

#include <iostream>

#include "fodom/synthetic/synthetic.h"
#include "fodom/unscented/unscented.h"
namespace fodom {

void Odom::propagate(const InputType& u, const Scalar dt) {
  // might want to look into error state KFs

  StateType X_copy(X_);
  CovType S_Copy(S_);
  // UT
  UnscentedTransform ut;
  ut.weight_scheme_ = WeightScheme::HB;
  SigmaPoints<StateType> spoints;
  ut.toUnscented(X_, S_, spoints);
  for (size_t i = 0; i < spoints.points.size(); ++i) {
    auto& point = spoints.points.at(i);
    point = IMUOutput::fwd(point, u, dt);
  }
  ut.toMeanCov(spoints, X_, S_);
}

void Odom::process_IMU(const IMUMeasurement<double>& meas) {
  std::cout << std::fixed << meas.stamp << "\n";
  // TODO IMU correction
}
}  // namespace fodom