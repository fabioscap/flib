#include "fodom/model.h"

namespace fodom {

/* IMUOutput::TangentType IMUOutput::vels(const IMUOutput::InputType& u,
                                       Scalar dt) const {
  IMUOutput::TangentType out;
  out.setZero();

  out.head<3>() = om();
  out.segment<3>(3) = v();  // + 0.5 * (R() * a() + g()) * dt;
  out.segment<3>(6) = R() * a() + g();
  out.segment<6>(9) = u.head<6>();
  // out.segment<3>(15).setZero();
  out.segment<6>(18) = u.tail<6>();

  return out;
} */

IMUOutput& IMUOutput::fwd(const IMUOutput::InputType& u, Scalar dt) {
  p() += v() * dt;
  v() += (R() * a() + g()) * dt;
  R() *= flib::math::expMapSO3<Scalar>(om() * dt);
  flib::math::fixRotation(R());

  // std::cout << (om() * dt).transpose() << "\n";
  // boxplus_inplace(dt * vels(u, dt));

  return *this;
}

}  // namespace fodom
