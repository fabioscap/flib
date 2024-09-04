#include "fodom/manifold.h"

namespace fodom {

ManifoldSO3::ManifoldSO3() : BaseType(DataType::Identity()) {}
ManifoldSO3::ManifoldSO3(const DataType& data) : BaseType(data) {}

ManifoldSO3 ManifoldSO3::boxplus(const TangentType& dsp) const {
  DataType R = data * expMapSO3(dsp);
  fixRotation(R);
  return ManifoldSO3(R);
}
ManifoldSO3::TangentType ManifoldSO3::boxminus(const ManifoldSO3& to) const {
  DataType R = to.data.transpose() * data.transpose();
  return logMapSO3(R.eval());
}

ManifoldSO3Q::ManifoldSO3Q() : BaseType(DataType::Identity()) {}
ManifoldSO3Q::ManifoldSO3Q(const DataType& data) : BaseType(data) {}

ManifoldSO3Q ManifoldSO3Q::boxplus(const TangentType& dsp) const {
  Scalar n = dsp.squaredNorm();
  if (n > Scalar(1.)) {
    return ManifoldSO3Q(Matrix3_<Scalar>::Identity());
  }
  Scalar w(sqrt(Scalar(1.) - n));
  Eigen::Quaternion<Scalar> full_q(w, dsp[0], dsp[1], dsp[2]);
  return ManifoldSO3Q(data * full_q.toRotationMatrix());
}
ManifoldSO3Q::TangentType ManifoldSO3Q::boxminus(const ManifoldSO3Q& to) const {
  Vector3_<Scalar> nq;
  Eigen::Quaternion<Scalar> q = r2q<Scalar>(to.data.transpose() * data);
  q.normalize();
  if (q.w() >= Scalar(0.)) {
    nq << q.x(), q.y(), q.z();
  } else {
    nq << -q.x(), -q.y(), -q.z();
  }
  return nq;
}

}  // namespace fodom