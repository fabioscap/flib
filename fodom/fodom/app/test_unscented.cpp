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

  auto u = MType::TangentType::Ones();
  x.perturb(u);

  std::cout << x.get<0>() << std::endl;
  std::cout << x.get<1>() << std::endl;

  std::function<void(MType&)> f = [](MType& x) {
    auto u = MType::TangentType::Ones();
    x.perturb(u);
    return;
  };

  propagate(x, C, f);

  // std::cout << x << "\n\n" << C << std::endl;
}