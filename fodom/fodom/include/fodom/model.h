#pragma once

// State transition models
#include "flib/manifold.h"
#include "fodom/common.h"

namespace fodom {

using namespace flib::manifold;

struct IMUOutput
    : public ManifoldComp_<SO3_<Scalar>,           // Orientation
                           Euclidean_<Scalar, 3>,  // Position
                           Euclidean_<Scalar, 3>,  // Velocity
                           Euclidean_<Scalar, 3>,  // Bias Gyro
                           Euclidean_<Scalar, 3>,  // Bias Acc
                           Euclidean_<Scalar, 3>,  // Gravity Vector global
                                                   // frame (Should be S2?)
                           Euclidean_<Scalar, 3>,  // Angular Velocity
                           Euclidean_<Scalar, 3>   // Acceleration
                           > {
  using InputType = Vector_<Scalar, 12>;

  // u: random walk bias noise (gyro, acc)
  //    gaussian noise on angular acceleration and jerk
  // TangentType vels(const InputType& u, Scalar dt) const;

  IMUOutput& fwd(const InputType& u, Scalar dt);

  // clang-format off
  inline SO3_<Scalar>::DataType& R() { return data.get<0>().data; }
  inline Euclidean_<Scalar,3>::DataType& p() { return data.get<1>().data; }
  inline Euclidean_<Scalar,3>::DataType& v() { return data.get<2>().data; }
  inline Euclidean_<Scalar,3>::DataType& bg() { return data.get<3>().data; }
  inline Euclidean_<Scalar,3>::DataType& ba() { return data.get<4>().data; }
  inline Euclidean_<Scalar,3>::DataType& g() { return data.get<5>().data; }
  inline Euclidean_<Scalar,3>::DataType& om() { return data.get<6>().data; }
  inline Euclidean_<Scalar,3>::DataType& a() { return data.get<7>().data; }
  inline const SO3_<Scalar>::DataType& R()const { return data.get<0>().data; }
  inline const Euclidean_<Scalar,3>::DataType& p()const { return data.get<1>().data; }
  inline const Euclidean_<Scalar,3>::DataType& v()const { return data.get<2>().data; }
  inline const Euclidean_<Scalar,3>::DataType& bg()const { return data.get<3>().data; }
  inline const Euclidean_<Scalar,3>::DataType& ba()const { return data.get<4>().data; }
  inline const Euclidean_<Scalar,3>::DataType& g()const { return data.get<5>().data; }
  inline const Euclidean_<Scalar,3>::DataType& om()const { return data.get<6>().data; }
  inline const Euclidean_<Scalar,3>::DataType& a()const { return data.get<7>().data; }
  // clang-format on
};

}  // namespace fodom
