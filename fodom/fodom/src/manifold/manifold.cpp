#include "manifold.h"

using namespace manifold;

int main() {
  using Eucl2 = Euclidean_<2>;
  using Eucl3 = Euclidean_<3>;
  using Eucl4 = Euclidean_<4>;

  // using MType = ManifoldComp_<Eucl3, Eucl2>;
  using MType = Eucl2;
  std::cout << "----\n";
  MType x;
  std::cout << x << "\n";
  MType::TangentType u = MType::TangentType::Ones();

  std::cout << "----\n";

  x.boxplus(u);
  std::cout << x << "\n";

  std::cout << "----\n";
  MType x2{x + u};
  std::cout << x2 << "\n";

  return 0;
}
