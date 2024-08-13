#include "fodom/model.h"

namespace fodom {

namespace IMUOutput {
StateType::TangentType vels(const StateType& X, const InputType& u) {
  StateType::TangentType out;

  out.head<3>() = X.get<6>().data();
  out.segment<3>(3) = X.get<2>().data();
  out.segment<3>(6) = X.get<0>().data() * X.get<6>().data() + X.get<5>().data();
  out.segment<6>(9) = u.head<6>();
  out.segment<3>(15).setZero();
  out.segment<6>(18) = u.tail<6>();

  return out;
}

StateType fwd(const StateType& X, const InputType& u, Scalar dt) {
  return X.boxplus(dt * vels(X, u));
}

MatrixN_<Scalar, StateType::dim> fwdJ(const StateType& X, Scalar dt) {}

// set initial conditions??
void setGrav(StateType& X, const Vector3_<Scalar>& g) { X.get<5>().data() = g; }
}  // namespace IMUOutput

class StateType;  // explicit instantiation

}  // namespace fodom
