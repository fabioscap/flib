#pragma once

// State transition models

#include <fodom/manifold.h>

namespace fodom {

struct IMUOutput : public ManifoldComp_<ManifoldSO3,    // Orientation
                                        Euclidean_<3>,  // Position
                                        Euclidean_<3>,  // Velocity
                                        Euclidean_<3>,  // Bias Gyro
                                        Euclidean_<3>,  // Bias Acc
                                        Euclidean_<3>,  // Gravity Vector global
                                                        // frame (Should be S2?)
                                        Euclidean_<3>,  // Angular Velocity
                                        Euclidean_<3>   // Acceleration
                                        > {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using BaseType = ThisType;
  using ThisType = IMUOutput;
  using InputType = Vector_<Scalar, 12>;

  inline IMUOutput() = default;
  /*   inline IMUOutput(const BaseType& other) : BaseType(other) {}

    ThisType& operator=(const BaseType& other) {
      this->BaseType::operator=(other);
      return *this;
    } */

  // u: random walk bias noise (gyro, acc)
  //    gaussian noise on angular acceleration and jerk
  static TangentType vels(const ThisType& X, const InputType& u);

  static ThisType fwd(const ThisType& X, const InputType& u, Scalar dt);

  inline ManifoldSO3::DataType& R() { return get<0>().data; }
  inline Euclidean_<3>::DataType& p() { return get<1>().data; }
  inline Euclidean_<3>::DataType& v() { return get<2>().data; }
  inline Euclidean_<3>::DataType& bg() { return get<3>().data; }
  inline Euclidean_<3>::DataType& ba() { return get<4>().data; }
  inline Euclidean_<3>::DataType& g() { return get<5>().data; }
  inline Euclidean_<3>::DataType& om() { return get<6>().data; }
  inline Euclidean_<3>::DataType& a() { return get<7>().data; }
};

}  // namespace fodom
