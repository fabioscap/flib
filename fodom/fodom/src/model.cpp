#include "fodom/model.h"

namespace fodom {

IMUOutput::TangentType IMUOutput::vels(const IMUOutput::InputType& u) const {
  IMUOutput::TangentType out;

  out.head<3>() = om();
  out.segment<3>(3) = v();
  out.segment<3>(6) = R() * om() + g();
  out.segment<6>(9) = u.head<6>();
  out.segment<3>(15).setZero();
  out.segment<6>(18) = u.tail<6>();

  return out;
}

void IMUOutput::fwd(const IMUOutput::InputType& u, Scalar dt) {
  boxplus_inplace(dt * vels(u));
}

}  // namespace fodom
