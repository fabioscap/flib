#include "flib/manifold.h"

namespace flib_old {
using namespace flib::manifold;
// just some helper functions
template <typename TupleType, size_t Idx>
constexpr size_t dim_f();
template <typename TupleType, size_t Idx>
constexpr size_t off_f();

template <typename... Manifolds_>
struct ManifoldComp_ : public ManifoldBase_<ManifoldComp_<Manifolds_...>,
                                            std::tuple<Manifolds_...>,
                                            (Manifolds_::dim + ... + 0)> {
  using DataType = std::tuple<Manifolds_...>;
  using BaseType =
      ManifoldBase_<ManifoldComp_, DataType, (Manifolds_::dim + ... + 0)>;

  using BaseType::ManifoldBase_;
  using TangentType = typename BaseType::TangentType;

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

void basic_test() {
  using namespace flib::manifold;
  using E = Euclidean_<double, 3>;

  E e1;

  E e2(Eigen::Vector3d{4, 5, 6});

  E::TangentType d = e1.compare(e2);

  e1.perturb(d);

  auto x = e1.boxplus(d);
  e1.boxplus(d, x);

  std::cout << x << std::endl;

  SO3_<double> R;

  SO3_<double>::TangentType om{1.0, 0.0, 1.0};

  R.perturb(om);

  auto R2(R);

  auto dr = R.compare(R2.boxplus(om));

  std::cout << R << "\n" << dr << std::endl;

  using CompType = ManifoldComp_<Euclidean_<double, 10>, SO3_<double>,
                                 Euclidean_<double, 3>, SO3_<double>>;
  CompType X, X2;

  CompType::TangentType xt = CompType::TangentType::Ones();

  X2.perturb(xt);

  std::cout << X2 << std::endl;

  std::cout << X.compare(X) << std::endl;
}

#include "nanobench.h"

template <typename F>
void benchmark(F&& f, const std::string& name) {
  using namespace ankerl;
  nanobench::Bench()
      .run(name,
           [&] {
             volatile auto result = f();
             nanobench::doNotOptimizeAway(result);
             ;
           })
      .relative(true);
}

int main() {
  using namespace flib::manifold;
  // basic_test();
  using EType = SO3_<double>;
  using MType = ManifoldComp_<EType, EType, EType, EType, EType, EType, EType>;
  using MTypeTuple =
      flib_old::ManifoldComp_<EType, EType, EType, EType, EType, EType, EType>;

  MType R;
  MType::TangentType u = MType::TangentType::Ones();

  MTypeTuple Rt;
  auto ut = Eigen::Vector<double, 70>::Ones();

  benchmark(
      [&] {
        R.perturb(u);
        return std::ref(R);
      },
      "1");
  benchmark(
      [&] {
        Rt.perturb(u);
        return std::ref(Rt);
      },
      "2");

  return 0;
}