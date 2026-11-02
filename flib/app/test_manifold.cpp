#include "flib/manifold.h"
// #include "synthetic/synthetic.h"

namespace flib_old {
using namespace flib::manifold;
// just some helper functions
template <typename TupleType, size_t Idx>
constexpr size_t dim_f();
template <typename TupleType, size_t Idx>
constexpr size_t off_f();

template <typename Derived, typename DataType_, size_t dim_>
struct ManifoldBase_ {
  using DataType = DataType_;
  DataType data;

  static constexpr size_t dim = dim_;
  using TangentType = Eigen::Vector<double, dim>;

  template <typename D>
  Derived boxplus(const Eigen::MatrixBase<D>& dsp) const {
    Derived out(*static_cast<const Derived*>(this));
    out.perturb(dsp);
    return out;
  }
};

template <typename... Manifolds_>
struct ManifoldComp_ : public ManifoldBase_<ManifoldComp_<Manifolds_...>,
                                            std::tuple<Manifolds_...>,
                                            (Manifolds_::dim + ... + 0)> {
  using DataType = std::tuple<Manifolds_...>;
  using BaseType =
      ManifoldBase_<ManifoldComp_, DataType, (Manifolds_::dim + ... + 0)>;

  using Scalar = double;
  using TangentType =
      typename Eigen::Vector<Scalar, (Manifolds_::dim + ... + 0)>;

  template <size_t I>
  static constexpr size_t dimi = dim_f<DataType, I>();
  template <size_t I>
  static constexpr size_t offi = off_f<DataType, I>();

  template <typename Derived, std::size_t... I>
  void perturb_impl(const Eigen::MatrixBase<Derived>& dsp,
                    std::index_sequence<I...>) {
    return (
        std::get<I>(this->data).perturb(dsp.template segment<dimi<I>>(offi<I>)),
        ...);
  }

  template <typename Derived>
  void perturb(const Eigen::MatrixBase<Derived>& dsp) {
    return perturb_impl(dsp, std::index_sequence_for<Manifolds_...>{});
  }

  template <std::size_t... I>
  void compare_impl(const ManifoldComp_& other, TangentType& out,
                    std::index_sequence<I...>) const {
    ((out.template segment<dimi<I>>(offi<I>) =
          std::get<I>(this->data).compare(std::get<I>(other.data))),
     ...);
  }

  TangentType compare(const ManifoldComp_& other) const {
    TangentType out;
    compare_impl(other, out, std::index_sequence_for<Manifolds_...>{});
    return out;
  }

  inline static const std::string name = "comp";
};

template <typename Derived, typename DataType, size_t dim>
std::ostream& operator<<(std::ostream& os,
                         const ManifoldBase_<Derived, DataType, dim>& m) {
  os << m.data;
  return os;
}

// definitions
template <typename TupleType, size_t Idx>
constexpr size_t dim_f() {
  return std::tuple_element_t<Idx, TupleType>::dim;
}

template <typename TupleType, size_t Idx>
constexpr size_t off_f() {
  if constexpr (Idx < 1)
    return 0;
  else
    return dim_f<TupleType, Idx - 1>() + off_f<TupleType, Idx - 1>();
}
}  // namespace flib_old

#include "nanobench.h"

// one shared Bench so .relative() compares all entries against the first one
inline ankerl::nanobench::Bench& bench() {
  static ankerl::nanobench::Bench b =
      ankerl::nanobench::Bench().relative(true).minEpochIterations(40000);
  return b;
}

template <typename F>
void benchmark(F&& f, const std::string& name) {
  bench().run(name, [&] {
    volatile auto result = f();
    ankerl::nanobench::doNotOptimizeAway(result);
  });
}

int main() {
  using namespace flib::manifold;
  using EType = Euclidean_<double, 3>;
  using MType = ManifoldComp_<EType, EType, EType, EType, EType, EType, EType>;
  using MTypeTuple =
      flib_old::ManifoldComp_<EType, EType, EType, EType, EType, EType, EType>;

  MType R, R2;
  MType::TangentType u = MType::TangentType::Ones();

  MTypeTuple Rt, Rt2;
  auto ut = Eigen::Vector<double, 21>::Ones();

  benchmark(
      [&] {
        R.perturb(u);
        return std::ref(R);
      },
      "no tuple perturb");
  benchmark(
      [&] {
        Rt.perturb(u);
        return std::ref(Rt);
      },
      "tuple perturb");

  R.perturb(u);
  Rt.perturb(ut);

  benchmark(
      [&] {
        u = R.compare(R2.boxplus(u));
        return std::ref(u);
      },
      "no tuple compare");

  benchmark(
      [&] {
        u = Rt.compare(Rt2.boxplus(u));
        return std::ref(u);
      },
      "tuple compare");

  return 0;
}