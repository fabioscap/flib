#include "fodom/manifold.h"

namespace fodom {

ManifoldSO3::ManifoldSO3() : BaseType(DataType::Identity()) {}
ManifoldSO3::ManifoldSO3(const DataType& data) : BaseType(data) {}

ManifoldSO3 ManifoldSO3::boxplus(const TangentType& dsp) const {
  DataType R = data() * expMapSO3(dsp);
  fixRotation(R);
  return ManifoldSO3(R);
}
ManifoldSO3::TangentType ManifoldSO3::boxminus(const ManifoldSO3& to) const {
  DataType R = to.data().transpose() * data_.transpose();
  return logMapSO3(R.eval());
}

ManifoldSO2::ManifoldSO2() : BaseType(0.0) {}
ManifoldSO2::ManifoldSO2(const DataType& data) : BaseType(data) {}

ManifoldSO2 ManifoldSO2::boxplus(const TangentType& dsp) const {
  Scalar s = std::sin(data() + dsp(0));
  Scalar c = std::cos(data() + dsp(0));
  return ManifoldSO2(std::atan2(s, c));
}

ManifoldSO2::TangentType ManifoldSO2::boxminus(const ManifoldSO2& to) const {
  Scalar s = std::sin(data() - to.data());
  Scalar c = std::cos(data() - to.data());
  return TangentType(std::atan2(s, c));
}
}