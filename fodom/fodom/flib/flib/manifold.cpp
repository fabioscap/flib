#include "manifold.h"

#include "unscented.h"

using namespace flib;
using namespace manifold;
using namespace unscented;

int main() {
  using Eucl2 = Euclidean_<2>;
  using Eucl3 = Euclidean_<3>;
  using Eucl4 = Euclidean_<4>;

  // using MType = ManifoldComp_<Eucl3, Eucl2>;
  using MType = Eucl2;

  MType x;

  x.boxplus_inplace(MType::TangentType::Ones());

  auto f = [](MType& x) { x.data *= 2.0; };
  UnscentedTransform<MType>::CovType c;
  c.setIdentity();

  std::cout << x << "\n";
  std::cout << c << "\n";
  propagate<MType>(x, c, f);

  std::cout << x << "\n";
  std::cout << c << "\n";
  return 0;
}
