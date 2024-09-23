#include "manifold.h"

#include "unscented.h"

int main() {
  using namespace flib::manifold;
  using namespace flib::unscented;

  using MType = ManifoldSO3;

  MType X;
  flib::math::MatrixN_<flib::Scalar, 3> S =
      1 * flib::math::MatrixN_<flib::Scalar, 3>::Identity();
  std::cout << S << "\n";
  std::cout << S.determinant() << "\n";
  propagate<MType>(X, S, [](MType& X) {
    X.boxplus_inplace(0.1 * Eigen::Vector<flib::Scalar, 3>{1, 0, 0});
  });
  std::cout << X << "\n";
  std::cout << S << "\n";
  std::cout << S.determinant() << "\n";

  return 0;
}