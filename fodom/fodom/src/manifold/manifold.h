#include <Eigen/Dense>
#include <iostream>
#include <utility>

namespace manifold {

constexpr size_t log = false;

// print a tuple. move it elsewhere
template <typename... Types>
std::ostream& operator<<(std::ostream& os, const std::tuple<Types...>& t) {
  std::apply([&os](const auto&... args) { ((os << args << "\n"), ...); }, t);
  return os;
}

template <typename Derived, typename DataType_>
struct ManifoldBase_ {
  using DataType = DataType_;
  DataType data;

  template <typename T>
  ManifoldBase_(T&& d) : data(std::forward<T>(d)) {
    if constexpr (log) std::cout << Derived::name << "MBase data ctor\n";
  }

  ManifoldBase_(const ManifoldBase_& other) : data(other.data) {
    if constexpr (log) std::cout << Derived::name << "MBase copy ctor\n";
  }
  ManifoldBase_(ManifoldBase_&& other) noexcept : data(std::move(other.data)) {
    if constexpr (log) std::cout << Derived::name << "MBase move ctor\n";
  }

  ManifoldBase_() {
    if constexpr (log) std::cout << Derived::name << "MBase def  ctor\n";
  }
};

// just some helper functions
template <typename TupleType, size_t Idx>
constexpr size_t dim_f();
template <typename TupleType, size_t Idx>
constexpr size_t off_f();

template <typename... Manifolds_>
struct ManifoldComp_ : public ManifoldBase_<ManifoldComp_<Manifolds_...>,
                                            std::tuple<Manifolds_...>> {
  using DataType = std::tuple<Manifolds_...>;
  using BaseType = ManifoldBase_<ManifoldComp_, DataType>;

  using BaseType::ManifoldBase_;

  static constexpr size_t dim = (Manifolds_::dim + ... + 0);

  template <size_t I>
  static constexpr size_t dimi = dim_f<DataType, I>();
  template <size_t I>
  static constexpr size_t offi = off_f<DataType, I>();

  template <typename Derived, std::size_t... I>
  auto boxplus_impl(const Eigen::MatrixBase<Derived>& dsp,
                    std::index_sequence<I...>) const {
    return std::make_tuple(
        std::get<I>(this->data)
            .boxplus(dsp.template segment<dimi<I>>(offi<I>))...);
  }

  template <typename Derived>
  auto boxplus(const Eigen::MatrixBase<Derived>& dsp) const {
    return boxplus_impl(dsp, std::index_sequence_for<Manifolds_...>{});
  }

  inline static const std::string name = "comp";
};

template <size_t dim_>
struct Euclidean_
    : public ManifoldBase_<Euclidean_<dim_>, Eigen::Vector<double, dim_>> {
  static constexpr size_t dim = dim_;
  using DataType = Eigen::Vector<double, dim>;
  using BaseType = ManifoldBase_<Euclidean_, DataType>;
  using BaseType::ManifoldBase_;

  Euclidean_() : BaseType(std::move(DataType::Ones())) {}

  template <typename T>
  auto boxplus(const Eigen::MatrixBase<T>& dsp) const {
    return this->data + dsp;
  }

  inline static const std::string name = "eucl";
};

template <typename Derived, typename DataType>
std::ostream& operator<<(std::ostream& os,
                         const ManifoldBase_<Derived, DataType>& m) {
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

}  // namespace manifold