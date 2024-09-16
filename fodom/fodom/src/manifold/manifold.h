#include <Eigen/Dense>
#include <iostream>
#include <utility>

namespace manifold {

constexpr size_t log = true;

// print a tuple. move it elsewhere
template <typename... Types>
std::ostream& operator<<(std::ostream& os, const std::tuple<Types...>& t) {
  std::apply([&os](const auto&... args) { ((os << args << "\n"), ...); }, t);
  return os;
}

template <typename Derived, typename DataType_, size_t dim_>
struct ManifoldBase_ {
  using DataType = DataType_;
  DataType data;

  static constexpr size_t dim = dim_;
  using TangentType = Eigen::Vector<double, dim>;

  // binds to anything that can be used to construct
  // data, and hopefully nothing else
  template <typename T>
  ManifoldBase_(T&& d) : data(std::forward<T>(d)) {
    if constexpr (log) std::cout << Derived::name << "MBase data ctor\n";
  }

  template <typename D>
  Derived operator+(const Eigen::MatrixBase<D>& dsp) const {
    Derived out(*static_cast<const Derived*>(this));
    out.boxplus(dsp);
    return out;  // NRVO
  }

  template <typename D>
  Derived& operator+=(const Eigen::MatrixBase<D>& dsp) {
    static_cast<Derived*>(this)->boxplus(dsp);
    return *this;
  }

  // not sure if needed, maybe replace with error
  // (if this is called, Derived did not implement boxplus)
  template <typename D>
  void boxplus(const Eigen::MatrixBase<D>& dsp) {
    if constexpr (log) std::cout << Derived::name << "boxplus\n";
    return static_cast<Derived*>(this)->boxplus(dsp);
  }

  // --- //
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
                                            std::tuple<Manifolds_...>,
                                            (Manifolds_::dim + ... + 0)> {
  using DataType = std::tuple<Manifolds_...>;
  using BaseType =
      ManifoldBase_<ManifoldComp_, DataType, (Manifolds_::dim + ... + 0)>;

  using BaseType::ManifoldBase_;

  template <size_t I>
  static constexpr size_t dimi = dim_f<DataType, I>();
  template <size_t I>
  static constexpr size_t offi = off_f<DataType, I>();

  template <typename Derived, std::size_t... I>
  void boxplus_impl(const Eigen::MatrixBase<Derived>& dsp,
                    std::index_sequence<I...>) {
    return (
        std::get<I>(this->data).boxplus(dsp.template segment<dimi<I>>(offi<I>)),
        ...);
  }

  template <typename Derived>
  void boxplus(const Eigen::MatrixBase<Derived>& dsp) {
    return boxplus_impl(dsp, std::index_sequence_for<Manifolds_...>{});
  }

  inline static const std::string name = "comp";
};

template <size_t dim_>
struct Euclidean_ : public ManifoldBase_<Euclidean_<dim_>,
                                         Eigen::Vector<double, dim_>, dim_> {
  static constexpr size_t dim = dim_;
  using DataType = Eigen::Vector<double, dim>;
  using BaseType = ManifoldBase_<Euclidean_, DataType, dim_>;
  using BaseType::ManifoldBase_;

  Euclidean_() : BaseType(DataType::Zero()) {}

  // in place!
  template <typename Derived>
  void boxplus(const Eigen::MatrixBase<Derived>& dsp) {
    this->data += dsp;
  }

  inline static const std::string name = "eucl";
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

}  // namespace manifold