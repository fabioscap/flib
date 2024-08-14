#include "fodom/odom.h"

#include <iostream>
#include <sstream>

#include "fodom/unscented.h"
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

  // add process covariance

  stamp_ += dt;
}

void Odom::process_IMU(const IMUMeasurement<double>& meas) {
  // propagate until meas.stamp
  log_function_("ehpporcodio!");
  log_function_(std::to_string(stamp_));
  log_function_(std::to_string(meas.stamp));
  if (stamp_) {
    if (stamp_ > meas.stamp + dt_min_) {
      log_function_("agg sburrat");
      return;
    } else
      while (stamp_ < meas.stamp - dt_min_) {
        propagate(InputType::Zero(), dt_min_);
      }
  }
  stamp_ = meas.stamp;

  auto acc_c = meas.lin_acc - X_.get<4>().data();
  auto ang_c = meas.ang_vel - X_.get<3>().data();

  Vector_<Scalar, 6> res;
  res << X_.get<6>().data() - ang_c, X_.get<7>().data() - acc_c;

  MatrixNM_<Scalar, 6, IMUOutput::StateType::dim> J =
      MatrixNM_<Scalar, 6, IMUOutput::StateType::dim>::Zero();
  J.block<6, 6>(0, 9) = MatrixN_<Scalar, 6>::Identity();
  J.block<6, 6>(0, 18) = MatrixN_<Scalar, 6>::Identity();
  auto K = S_ * J.transpose() *
           (J * S_ * J.transpose() + 1e-5 * MatrixN_<Scalar, 6>::Identity())
               .inverse();

  S_ *= (CovType::Identity() - K * J);
  X_ = X_.boxplus(K * res);

  std::ostringstream s;
  s << K * res;
  log_function_(s.str());
}
}  // namespace fodom