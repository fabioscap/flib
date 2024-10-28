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

  sw_->tic();
  // UT
  unscented::propagate<ModelType>(X_, S_,
                                  [&u, &dt](ModelType& x) { x.fwd(u, dt); });
  // auto d = sw_->toc();

  // add process covariance
  // X_.fwd(u, dt);
  // S_ = CovType::Identity();
  // S_.block<6, 6>(9, 9) += MatrixN_<Scalar, 6>::Identity() * 1e-5;
  // S_.block<6, 6>(18, 18) += MatrixN_<Scalar, 6>::Identity() * 1e-5;
  stamp_ += dt;
}

void Odom::process_IMU(const IMUMeasurement<Scalar>& meas) {
  // propagate until meas.stamp
  // TODO if delta time too big then re initialize
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
  } else {
    stamp_ = meas.stamp;
  }
  auto acc_c = meas.lin_acc;
  auto ang_c = meas.ang_vel;

  /*   X_.a() = meas.lin_acc;
    X_.om() = meas.ang_vel; */

  Vector_<Scalar, 6> res;

  res << ang_c - X_.om(), acc_c - X_.a();

  MatrixNM_<Scalar, 6, IMUOutput::dim> J =
      MatrixNM_<Scalar, 6, IMUOutput::dim>::Zero();
  // J.block<6, 6>(0, 9) = MatrixN_<Scalar, 6>::Identity();
  J.block<6, 6>(0, 18) = MatrixN_<Scalar, 6>::Identity();
  auto Y = J * S_ * J.transpose() + 1e-3 * MatrixN_<Scalar, 6>::Identity();
  auto K = S_ * J.transpose() * Y.inverse();

  S_ -= K * J * S_;
  X_.boxplus_inplace(K * res);
}

void Odom::process_cloud(const std::vector<Point<Scalar>>& cloud) {
  std::ostringstream s;
  s << std::fixed << cloud.at(cloud.size() - 1).stamp << '\n';
  log_function_(s.str());
}
}  // namespace fodom