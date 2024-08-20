#include "fodom/odom.h"

#include <iostream>
#include <sstream>

#include "fodom/unscented.h"

namespace fodom {

Odom::Odom(void (*f)(const std::string&),
           const std::shared_ptr<StopWatchBase>& sw)
    : log_function_(f), sw_(sw) {};

void Odom::propagate(const InputType& u, const Scalar dt) {
  // might want to look into error state KFs

  // UT
  UnscentedTransform ut;
  ut.weight_scheme_ = WeightScheme::HB;
  SigmaPoints<ModelType> spoints;
  ut.toUnscented(X_, S_, spoints);
  for (size_t i = 0; i < spoints.points.size(); ++i) {
    auto& point = spoints.points.at(i);
    point = ModelType::fwd(point, u, dt);
  }
  ut.toMeanCov(spoints, X_, S_);

  // add process covariance

  stamp_ += dt;
}

void Odom::process_IMU(const IMUMeasurement<double>& meas) {
  // propagate until meas.stamp
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

  auto acc_c = meas.lin_acc - X_.get<4>().data;
  auto ang_c = meas.ang_vel - X_.get<3>().data;

  Vector_<Scalar, 6> res;
  res << X_.get<6>().data - ang_c, X_.get<7>().data - acc_c;

  MatrixNM_<Scalar, 6, IMUOutput::dim> J =
      MatrixNM_<Scalar, 6, IMUOutput::dim>::Zero();
  J.block<6, 6>(0, 9) = MatrixN_<Scalar, 6>::Identity();
  J.block<6, 6>(0, 18) = MatrixN_<Scalar, 6>::Identity();
  auto K = S_ * J.transpose() *
           (J * S_ * J.transpose() + 1e-5 * MatrixN_<Scalar, 6>::Identity())
               .inverse();

  S_ *= (CovType::Identity() - K * J);
  X_ = X_.boxplus(K * res);

  /*   std::ostringstream s;
    s << K * res;
    log_function_(s.str()); */
}
}  // namespace fodom