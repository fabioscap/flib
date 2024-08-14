#pragma once

#include <Eigen/Dense>

namespace fodom {
template <typename Scalar_>
using VectorX_ = Eigen::Matrix<Scalar_, Eigen::Dynamic, 1>;

template <typename Scalar_, int Dim_>
using Vector_ = Eigen::Matrix<Scalar_, Dim_, 1>;

template <typename Scalar_>
using Vector3_ = Eigen::Matrix<Scalar_, 3, 1>;

template <typename Scalar_>
using MatrixX_ = Eigen::Matrix<Scalar_, Eigen::Dynamic, Eigen::Dynamic>;

template <typename Scalar_, int N_, int M_>
using MatrixNM_ = Eigen::Matrix<Scalar_, N_, M_>;

template <typename Scalar_, int N_>
using MatrixN_ = Eigen::Matrix<Scalar_, N_, N_>;

template <typename Scalar_>
using Matrix3_ = Eigen::Matrix<Scalar_, 3, 3>;

template <typename Scalar_>
using Isometry3_ = Eigen::Transform<Scalar_, 3, Eigen::Isometry>;

template <typename Scalar_>
inline Matrix3_<Scalar_> skew(const Vector3_<Scalar_> &v_) {
  Matrix3_<Scalar_> S;
  S << Scalar_(0.), -v_[2], v_[1], v_[2], Scalar_(0.), -v_[0], -v_[1], v_[0],
      Scalar_(0.);
  return S;
}

template <typename Scalar_>
inline Vector3_<Scalar_> logMapSO3(const Matrix3_<Scalar_> &R) {
  const Scalar_ &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  const Scalar_ &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  const Scalar_ &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  const Scalar_ tr = R.trace();
  const Scalar_ pi(M_PI);
  const Scalar_ two(2);

  Vector3_<Scalar_> omega;
  if (tr + Scalar_(1.0) < Scalar_(1e-10)) {
    if (abs(R33 + Scalar_(1.0)) > Scalar_(1e-5)) {
      omega = (pi / sqrt(two + two * R33)) *
              Vector3_<Scalar_>(R13, R23, Scalar_(1.0) + R33);
    } else if (abs(R22 + Scalar_(1.0)) > Scalar_(1e-5)) {
      omega = (pi / sqrt(two + two * R22)) *
              Vector3_<Scalar_>(R12, Scalar_(1.0) + R22, R32);
    } else {
      omega = (pi / sqrt(two + two * R11)) *
              Vector3_<Scalar_>(Scalar_(1.0) + R11, R21, R31);
    }
  } else {
    Scalar_ magnitude;
    const Scalar_ tr_3 = tr - Scalar_(3.0);
    if (tr_3 < Scalar_(-1e-7)) {
      Scalar_ theta = acos((tr - Scalar_(1.0)) / two);
      magnitude = theta / (two * sin(theta));
    } else {
      magnitude = Scalar_(0.5) - tr_3 * tr_3 / Scalar_(12.0);
    }
    omega = magnitude * Vector3_<Scalar_>(R32 - R23, R13 - R31, R21 - R12);
  }
  return omega;
}

template <typename Scalar_>
inline Matrix3_<Scalar_> expMapSO3(const Vector3_<Scalar_> &omega_) {
  Matrix3_<Scalar_> R;
  Scalar_ theta_square = omega_.dot(omega_);
  Scalar_ theta = sqrt(theta_square);
  Matrix3_<Scalar_> W = skew(omega_);
  Matrix3_<Scalar_> K = W / theta;
  if (theta_square < Scalar_(1e-8)) {
    R = Matrix3_<Scalar_>::Identity() + W;
  } else {
    Scalar_ one_minus_cos =
        Scalar_(2) * sin(theta / Scalar_(2)) * sin(theta / Scalar_(2));
    R = Matrix3_<Scalar_>::Identity() + sin(theta) * K + one_minus_cos * K * K;
  }
  return R;
}

template <typename MatrixType_>
void fixRotation(MatrixType_ &R) {
  MatrixType_ E = R.transpose() * R;
  E.diagonal().array() -= 1;
  R -= 0.5 * R * E;
}

}  // namespace fodom