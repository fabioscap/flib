#include "manifold.h"

using namespace manifold;

int main() {
  using Eucl2 = Euclidean_<2>;
  using Eucl3 = Euclidean_<3>;
  using Eucl4 = Euclidean_<4>;

  using MType = ManifoldComp_<Eucl3, Eucl2>;
  // using MType = Eucl2;
  std::cout << "----\n";
  MType x;

  std::cout << MType::dimi<0> << " " << MType::dimi<1> << "\n";
  std::cout << MType::offi<0> << " " << MType::offi<1> << "\n";
  return 0;
}
