#include "fodom/odom.h"

#include <iostream>
#include <sstream>

#include "flib/unscented.h"

namespace fodom {

using namespace flib;

Odom::Odom(void (*f)(const std::string&),
           const std::shared_ptr<StopWatchBase>& sw)
    : log_function_(f), sw_(sw) {};

void Odom::propagate(const InputType& u, const Scalar dt) {
  // might want to look into error state KFs

  // UT
  unscented::propagate<ModelType>(X_, S_,
                                  [&u, &dt](ModelType& x) { x.fwd(u, dt); });
  // add process covariance
  S_ += 1e-100 * dt * CovType::Identity();

  stamp_ += dt;
}

void Odom::process_IMU(const IMUMeasurement<double>& meas) {
  // propagate until meas.stamp
  if (stamp_) {
    if (stamp_ > meas.stamp + dt_min_) {
      log_function_("agg sburrat");
      return;
    } else
      while (stamp_ < meas.stamp) {
        if (meas.stamp - stamp_ > dt_max_)
          propagate(InputType::Zero(), dt_max_);
        else
          propagate(InputType::Zero(), meas.stamp - stamp_);
      }
  }
  stamp_ = meas.stamp;

  auto acc_c = meas.lin_acc - X_.ba();
  auto ang_c = meas.ang_vel - X_.bg();

  Vector_<Scalar, 6> res;

  res << X_.om() - ang_c, X_.a() - acc_c;
  res *= 0.0;

  MatrixNM_<Scalar, 6, IMUOutput::dim> J =
      MatrixNM_<Scalar, 6, IMUOutput::dim>::Zero();
  J.block<6, 6>(0, 9) = MatrixN_<Scalar, 6>::Identity();
  J.block<6, 6>(0, 18) = MatrixN_<Scalar, 6>::Identity();
  auto K = S_ * J.transpose() *
           (J * S_ * J.transpose() + 1e-1 * MatrixN_<Scalar, 6>::Identity())
               .inverse();

  S_ *= (CovType::Identity() - K * J);
  X_.boxplus_inplace(K * res);
}
}  // namespace fodom