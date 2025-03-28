#include "flib/manifold.h"
#include "flib/unscented.h"

int main() {
  using namespace flib::manifold;
  using namespace flib::unscented;
  using MType = ManifoldComp_<SO3_<double>, Euclidean_<double, 3>>;

  MType x;

  UnscentedTransform<MType> ut;

  UnscentedTransform<MType>::CovType C =
      UnscentedTransform<MType>::CovType::Identity();

  std::function<void(MType&)> f = [](MType& x) {
    auto u = MType::TangentType::Ones();
    auto R = x.get<0>().data;
    x.get<0>().perturb(u.template head<3>());
    x.get<1>().perturb(R * u.template tail<3>());
    return;
  };

  propagate(x, C, f);

  std::cout << x << "\n\n" << C << std::endl;

  std::cout << C.determinant() << std::endl;
}