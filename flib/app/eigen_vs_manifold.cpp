// boxplus cost: hand-written Eigen structs vs flib manifolds of the same shape
#include <nanobench.h>

#include "flib/manifold.h"

using namespace flib::manifold;
using Scalar = double;
template <int N>
using Vec = Eigen::Vector<Scalar, N>;

// one 24-dim vector
struct eigFull {
  eigFull() : X(Vec<24>::Zero()) {}
  template <typename D>
  eigFull(const Eigen::MatrixBase<D>& data) : X(data) {}

  template <typename D>
  eigFull boxplus(const Eigen::MatrixBase<D>& u) const {
    return eigFull(X + u);
  }
  Vec<24> X;
};

// eight 3-dim blocks, written out by hand
struct eigBlock {
  eigBlock() {
    for (auto* x : {&X0, &X1, &X2, &X3, &X4, &X5, &X6, &X7}) x->setZero();
  }
  eigBlock(const Vec<3>& d0, const Vec<3>& d1, const Vec<3>& d2,
           const Vec<3>& d3, const Vec<3>& d4, const Vec<3>& d5,
           const Vec<3>& d6, const Vec<3>& d7)
      : X0(d0), X1(d1), X2(d2), X3(d3), X4(d4), X5(d5), X6(d6), X7(d7) {}

  template <typename D>
  eigBlock boxplus(const Eigen::MatrixBase<D>& u) const {
    return eigBlock(
        X0 + u.template head<3>(), X1 + u.template segment<3>(3),
        X2 + u.template segment<3>(6), X3 + u.template segment<3>(9),
        X4 + u.template segment<3>(12), X5 + u.template segment<3>(15),
        X6 + u.template segment<3>(18), X7 + u.template segment<3>(21));
  }
  Vec<3> X0, X1, X2, X3, X4, X5, X6, X7;
};

const Vec<24> u = Vec<24>::Ones();

template <typename T>
void benchmark(ankerl::nanobench::Bench& bench, const T& x,
               const std::string& name) {
  bench.run(name, [&] {
    T out{x.boxplus(u)};
    ankerl::nanobench::doNotOptimizeAway(out);
  });
}

int main() {
  using E3 = Euclidean_<Scalar, 3>;
  Euclidean_<Scalar, 24> eucl;
  ManifoldComp_<E3, E3, E3, E3, E3, E3, E3, E3> eucl_comp;
  eigFull eigfull;
  eigBlock eigblock;

  ankerl::nanobench::Bench bench;
  bench.title("boxplus").relative(true).minEpochIterations(100000);

  benchmark(bench, eigblock, "hand-written blocks (8x3)");
  benchmark(bench, eigfull, "hand-written full (24)");
  benchmark(bench, eucl, "Euclidean_<24>");
  benchmark(bench, eucl_comp, "ManifoldComp_<8x Euclidean_<3>>");

  return 0;
}
