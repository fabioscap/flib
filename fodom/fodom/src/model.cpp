#include "fodom/model.h"

namespace fodom {

IMUOutput::TangentType IMUOutput::vels(const ThisType& X, const InputType& u) {
  ThisType::TangentType out;

  out.head<3>() = X.get<6>().data;
  out.segment<3>(3) = X.get<2>().data;
  out.segment<3>(6) = X.get<0>().data * X.get<6>().data + X.get<5>().data;
  out.segment<6>(9) = u.head<6>();
  out.segment<3>(15).setZero();
  out.segment<6>(18) = u.tail<6>();

  return out;
}

IMUOutput IMUOutput::fwd(const ThisType& X, const InputType& u, Scalar dt) {
  return X.boxplus(dt * vels(X, u));
}

}  // namespace fodom
