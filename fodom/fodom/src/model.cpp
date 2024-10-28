#include "fodom/model.h"

namespace fodom {

IMUOutput& IMUOutput::fwd(const IMUOutput::InputType& u, Scalar dt) {
  p() += v() * dt;
  v() += (R() * a() + g()) * dt;
  R() *= flib::math::expMapSO3<Scalar>(om() * dt);
  flib::math::fixRotation(R());

  return *this;
}

}  // namespace fodom
