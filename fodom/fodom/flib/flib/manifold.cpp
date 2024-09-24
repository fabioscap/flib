#include "manifold.h"

#include "unscented.h"

int main() {
  using namespace flib::manifold;
  using namespace flib::unscented;

  using MType = ManifoldSO3;
  using Matrix3 = flib::math::MatrixN_<flib::Scalar, 3>;
  MType X;
  Matrix3 S = 0.1 * Matrix3::Identity();
  std::cout << S << "\n";
  std::cout << S.determinant() << "\n";
  propagate<MType>(X, S, [](MType& X) {
    X.data *= flib::math::Rx<flib::Scalar>(0.2);
    X.data *= flib::math::Rx<flib::Scalar>(0.2);
    X.data *= flib::math::Rx<flib::Scalar>(0.2);
    X.data *= flib::math::Rx<flib::Scalar>(0.2);
    X.data *= flib::math::Rx<flib::Scalar>(0.2);
  });
  std::cout << X << "\n";
  std::cout << S << "\n";
  std::cout << S.determinant() << "\n";

  return 0;
}