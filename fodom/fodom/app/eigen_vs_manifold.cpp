#include <nanobench.h>

#include <iostream>

#include "fodom/common.h"
#include "fodom/manifold.h"
#include "fodom/manifold2.h"
#include "fodom/utils.h"

using namespace fodom;

ChronoStopWatch sw;
static constexpr size_t N = 10000;
Eigen::Vector<Scalar, 24> u = Eigen::Vector<Scalar, 24>::Ones();
template <typename T>
double benchmark(T& x) {
  for (size_t i = 0; i < N; ++i) {
    x.boxplus(u);
  }
  size_t t{0};
  ankerl::nanobench::Bench().minEpochIterations(5000).run("boxplus", [&] {
    asm volatile("" ::"r"(1.23));
    ++t;
    T temp{x.boxplus(u)};
  });
  std::cout << t << "\n";
  return 0;
}
template <typename T>
double benchmark2(T& x) {
  for (size_t i = 0; i < N; ++i) {
    x.boxplus(u);
  }
  size_t t{0};
  ankerl::nanobench::Bench().minEpochIterations(5000).run("boxplus", [&] {
    asm volatile("" ::"r"(1.23));
    ++t;
    T temp{x + u};
  });
  std::cout << t << "\n";
  return 0;
}

template <typename T>
double run(T& x) {
  asm volatile("" ::"r"(1.23));
  std::cout << x << "\n\n";
  x.boxplus(u);
  return 0;
}

size_t xxd{0};

namespace manifold {}
struct eigFull {
  eigFull() : X(Vector_<Scalar, 24>::Zero()) {}
  template <typename D>
  eigFull(const Eigen::MatrixBase<D>& data) : X(data) {
    // xxd++;
  }

  template <typename D>
  inline eigFull boxplus(const Eigen::MatrixBase<D>& u) {
    ++xxd;
    return eigFull(X + u);
  }
  Vector_<Scalar, 24> X;
};

struct eigBlock {
  eigBlock()
      : X0(Vector_<Scalar, 3>::Zero()),
        X1(Vector_<Scalar, 3>::Zero()),
        X2(Vector_<Scalar, 3>::Zero()),
        X3(Vector_<Scalar, 3>::Zero()),
        X4(Vector_<Scalar, 3>::Zero()),
        X5(Vector_<Scalar, 3>::Zero()),
        X6(Vector_<Scalar, 3>::Zero()),
        X7(Vector_<Scalar, 3>::Zero()) {}

  template <typename Derived>
  eigBlock(const Eigen::MatrixBase<Derived>& d0,
           const Eigen::MatrixBase<Derived>& d1,
           const Eigen::MatrixBase<Derived>& d2,
           const Eigen::MatrixBase<Derived>& d3,
           const Eigen::MatrixBase<Derived>& d4,
           const Eigen::MatrixBase<Derived>& d5,
           const Eigen::MatrixBase<Derived>& d6,
           const Eigen::MatrixBase<Derived>& d7)
      : X0(d0), X1(d1), X2(d2), X3(d3), X4(d4), X5(d5), X6(d6), X7(d7) {
    // xxd++;
  }
  inline eigBlock boxplus(const Vector_<Scalar, 24>& u) {
    ++xxd;
    return eigBlock(
        X0 + u.template head<3>(), X1 + u.template segment<3>(3),
        X2 + u.template segment<3>(6), X3 + u.template segment<3>(9),
        X4 + u.template segment<3>(12), X5 + u.template segment<3>(15),
        X6 + u.template segment<3>(18), X7 + u.template segment<3>(21));
  }
  Vector_<Scalar, 3> X0;
  Vector_<Scalar, 3> X1;
  Vector_<Scalar, 3> X2;
  Vector_<Scalar, 3> X3;
  Vector_<Scalar, 3> X4;
  Vector_<Scalar, 3> X5;
  Vector_<Scalar, 3> X6;
  Vector_<Scalar, 3> X7;
};

int main() {
  Euclidean_<24> eucl;
  ManifoldComp_<Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
                Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>
      eucl_comp;
  manifold::Euclidean_<24> eucl2;
  manifold::ManifoldComp_<manifold::Euclidean_<3>, manifold::Euclidean_<3>,
                          manifold::Euclidean_<3>, manifold::Euclidean_<3>,
                          manifold::Euclidean_<3>, manifold::Euclidean_<3>,
                          manifold::Euclidean_<3>, manifold::Euclidean_<3>>
      eucl_comp2;

  eigFull eigfull;
  eigBlock eigblock;

  benchmark(eigblock);
  benchmark(eigfull);
  benchmark(eucl);
  benchmark(eucl_comp);
  benchmark2(eucl2);
  benchmark2(eucl_comp2);
  /*   run(eucl_comp);
    run(eucl_comp2); */

  return 0;
}