#pragma once

// State transition models
#include "flib/manifold.h"
#include "fodom/common.h"

namespace fodom {

using namespace flib::manifold;

struct IMUOutput : public ManifoldComp_<ManifoldSO3,    // Orientation
                                        Euclidean_<3>,  // Position
                                        Euclidean_<3>,  // Velocity
                                        Euclidean_<3>,  // Bias Gyro
                                        Euclidean_<3>,  // Bias Acc
                                        Euclidean_<3>,  // Gravity Vector global
                                        Euclidean_<3>,  // frame (Should be S2?)
                                        Euclidean_<3>,  // Angular Velocity
                                        Euclidean_<3>   // Acceleration
                                        > {
  using InputType = Vector_<Scalar, 12>;

  // u: random walk bias noise (gyro, acc)
  //    gaussian noise on angular acceleration and jerk
  TangentType vels(const InputType& u) const;

  void fwd(const InputType& u, Scalar dt);

  // clang-format off
  inline ManifoldSO3::DataType& R() { return std::get<0>(this->data).data; }
  inline Euclidean_<3>::DataType& p() { return std::get<1>(this->data).data; }
  inline Euclidean_<3>::DataType& v() { return std::get<2>(this->data).data; }
  inline Euclidean_<3>::DataType& bg() { return std::get<3>(this->data).data; }
  inline Euclidean_<3>::DataType& ba() { return std::get<4>(this->data).data; }
  inline Euclidean_<3>::DataType& g() { return std::get<5>(this->data).data; }
  inline Euclidean_<3>::DataType& om() { return std::get<6>(this->data).data; }
  inline Euclidean_<3>::DataType& a() { return std::get<7>(this->data).data; }
  inline const ManifoldSO3::DataType& R()const { return std::get<0>(this->data).data; }
  inline const Euclidean_<3>::DataType& p()const { return std::get<1>(this->data).data; }
  inline const Euclidean_<3>::DataType& v()const { return std::get<2>(this->data).data; }
  inline const Euclidean_<3>::DataType& bg()const { return std::get<3>(this->data).data; }
  inline const Euclidean_<3>::DataType& ba()const { return std::get<4>(this->data).data; }
  inline const Euclidean_<3>::DataType& g()const { return std::get<5>(this->data).data; }
  inline const Euclidean_<3>::DataType& om()const { return std::get<6>(this->data).data; }
  inline const Euclidean_<3>::DataType& a()const { return std::get<7>(this->data).data; }
  // clang-format on
};

}  // namespace fodom
