#pragma once

#include <Eigen/Dense>

namespace flib {
namespace math {
template <typename Scalar>
using VectorX_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

template <typename Scalar, int Dim_>
using Vector_ = Eigen::Matrix<Scalar, Dim_, 1>;

template <typename Scalar>
using Vector3_ = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar>
using MatrixX_ = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

template <typename Scalar, int N_, int M_>
using MatrixNM_ = Eigen::Matrix<Scalar, N_, M_>;

template <typename Scalar, int N_>
using MatrixN_ = Eigen::Matrix<Scalar, N_, N_>;

template <typename Scalar>
using Matrix3_ = Eigen::Matrix<Scalar, 3, 3>;

template <typename Scalar>
using Isometry3_ = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

template <typename Scalar, typename D>
inline Matrix3_<Scalar> skew(const Eigen::MatrixBase<D> &v_) {
  static_assert(
      std::is_same<typename Eigen::MatrixBase<D>::Scalar, Scalar>::value,
      "Scalar types do not match");
  static_assert(Eigen::MatrixBase<D>::RowsAtCompileTime == 3 &&
                Eigen::MatrixBase<D>::ColsAtCompileTime == 1);
  Matrix3_<Scalar> S;
  S << Scalar(0.), -v_[2], v_[1], v_[2], Scalar(0.), -v_[0], -v_[1], v_[0],
      Scalar(0.);
  return S;
}

template <typename Scalar, typename D>
inline Vector3_<Scalar> logMapSO3(const Eigen::MatrixBase<D> &R) {
  static_assert(
      std::is_same<typename Eigen::MatrixBase<D>::Scalar, Scalar>::value,
      "Scalar types do not match");
  static_assert(Eigen::MatrixBase<D>::RowsAtCompileTime == 3 &&
                Eigen::MatrixBase<D>::ColsAtCompileTime == 3);
  const Scalar &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  const Scalar &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  const Scalar &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  const Scalar tr = R.trace();
  const Scalar pi(M_PI);
  const Scalar two(2);

  Vector3_<Scalar> omega;
  if (tr + Scalar(1.0) < Scalar(1e-10)) {
    if (abs(R33 + Scalar(1.0)) > Scalar(1e-5)) {
      omega = (pi / sqrt(two + two * R33)) *
              Vector3_<Scalar>(R13, R23, Scalar(1.0) + R33);
    } else if (abs(R22 + Scalar(1.0)) > Scalar(1e-5)) {
      omega = (pi / sqrt(two + two * R22)) *
              Vector3_<Scalar>(R12, Scalar(1.0) + R22, R32);
    } else {
      omega = (pi / sqrt(two + two * R11)) *
              Vector3_<Scalar>(Scalar(1.0) + R11, R21, R31);
    }
  } else {
    Scalar magnitude;
    const Scalar tr_3 = tr - Scalar(3.0);
    if (tr_3 < Scalar(-1e-7)) {
      Scalar theta = acos((tr - Scalar(1.0)) / two);
      magnitude = theta / (two * sin(theta));
    } else {
      magnitude = Scalar(0.5) - tr_3 * tr_3 / Scalar(12.0);
    }
    omega = magnitude * Vector3_<Scalar>(R32 - R23, R13 - R31, R21 - R12);
  }
  return omega;
}

template <typename Scalar, typename D>
inline Matrix3_<Scalar> expMapSO3(const Eigen::MatrixBase<D> &omega_) {
  static_assert(
      std::is_same<typename Eigen::MatrixBase<D>::Scalar, Scalar>::value,
      "Scalar types do not match");
  static_assert(Eigen::MatrixBase<D>::RowsAtCompileTime == 3 &&
                Eigen::MatrixBase<D>::ColsAtCompileTime == 1);
  Scalar theta_square = omega_.dot(omega_);
  Scalar theta = sqrt(theta_square);
  Matrix3_<Scalar> W = skew<Scalar>(omega_);
  Matrix3_<Scalar> K = W / theta;
  if (theta_square < Scalar(1e-8)) {
    return Matrix3_<Scalar>::Identity() + W;
  } else {
    Scalar one_minus_cos =
        Scalar(2) * sin(theta / Scalar(2)) * sin(theta / Scalar(2));
    return Matrix3_<Scalar>::Identity() + sin(theta) * K +
           one_minus_cos * K * K;
  }
}

template <typename MatrixType_>
void fixRotation(MatrixType_ &R) {
  MatrixType_ E = R.transpose() * R;
  E.diagonal().array() -= 1;
  R -= 0.5 * R * E;
}

template <typename Scalar, typename D>
Eigen::Quaternion<Scalar> r2q(const Eigen::MatrixBase<D> &R_) {
  static_assert(
      std::is_same<typename Eigen::MatrixBase<D>::Scalar, Scalar>::value,
      "Scalar types do not match");
  static_assert(Eigen::MatrixBase<D>::RowsAtCompileTime == 3 &&
                Eigen::MatrixBase<D>::ColsAtCompileTime == 3);
  Eigen::Quaternion<Scalar> q;
  Scalar t = R_.trace();
  if (t > Scalar(0.)) {
    t = sqrt(t + Scalar(1.0));
    q.w() = Scalar(0.5) * t;
    t = Scalar(0.5) / t;
    q.x() = (R_.coeff(2, 1) - R_.coeff(1, 2)) * t;
    q.y() = (R_.coeff(0, 2) - R_.coeff(2, 0)) * t;
    q.z() = (R_.coeff(1, 0) - R_.coeff(0, 1)) * t;
  } else {
    int i = 0;
    if (R_.coeff(1, 1) > R_.coeff(0, 0)) {
      i = 1;
    }
    if (R_.coeff(2, 2) > R_.coeff(i, i)) {
      i = 2;
    }
    int j = (i + 1) % 3;
    int k = (j + 1) % 3;

    t = sqrt(R_.coeff(i, i) - R_.coeff(j, j) - R_.coeff(k, k) + Scalar(1.0));
    q.coeffs().coeffRef(i) = Scalar(0.5) * t;
    t = Scalar(0.5) / t;
    q.w() = (R_.coeff(k, j) - R_.coeff(j, k)) * t;
    q.coeffs().coeffRef(j) = (R_.coeff(j, i) + R_.coeff(i, j)) * t;
    q.coeffs().coeffRef(k) = (R_.coeff(k, i) + R_.coeff(i, k)) * t;
  }
  return q;
}
}  // namespace math
}  // namespace flib