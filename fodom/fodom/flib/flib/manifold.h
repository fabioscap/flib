#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <utility>

#include "common.h"
#include "math.h"

namespace flib {
namespace manifold {

constexpr bool log = false;

// print a tuple. move it elsewhere
template <typename... Types>
std::ostream& operator<<(std::ostream& os, const std::tuple<Types...>& t) {
  std::apply([&os](const auto&... args) { ((os << args << "\n"), ...); }, t);
  return os;
}

template <typename Derived, typename DataType_, size_t dim_>
struct ManifoldBase_ {
  // using Scalar_ = Scalar;
  using DataType = DataType_;
  DataType data;

  static constexpr size_t dim = dim_;
  using TangentType = Eigen::Vector<Scalar, dim>;

  // binds to anything that can be used to construct
  // data, and hopefully nothing else
  // Why does it not overshadow copy constructor?
  template <typename T>
  ManifoldBase_(T&& d) noexcept : data(std::forward<T>(d)) {
    if constexpr (log) std::cout << Derived::name << "MBase data ctor\n";
  }

  template <typename D>
  Derived& boxplus(const Eigen::MatrixBase<D>& dsp, Derived& out) const {
    out = *static_cast<const Derived*>(this);
    out.perturb(dsp);
    return out;
  }

  template <typename D>
  Derived boxplus(const Eigen::MatrixBase<D>& dsp) const {
    Derived out(*static_cast<const Derived*>(this));
    out.perturb(dsp);
    return out;
  }

  template <typename D>
  void boxplus_inplace(const Eigen::MatrixBase<D>& dsp) {
    static_cast<Derived*>(this)->perturb(dsp);
  }

  TangentType& boxminus(const Derived& other, TangentType& out) const {
    out = static_cast<const Derived*>(this)->compare(other);
    return out;
  }

  TangentType boxminus(const Derived& other) const {
    return static_cast<const Derived*>(this)->compare(other);
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

  ManifoldBase_& operator=(const ManifoldBase_& other) {
    if constexpr (log) std::cout << Derived::name << "MBase eq assgn\n";
    if (this != &other) {
      data = other.data;
    }
    return *this;
  }

  ManifoldBase_& operator=(ManifoldBase_&& other) noexcept {
    if constexpr (log) std::cout << Derived::name << "MBase eq mvassgn\n";
    data = std::move(other.data);
    return *this;
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

template <size_t dim_>
struct Euclidean_ : public ManifoldBase_<Euclidean_<dim_>,
                                         Eigen::Vector<Scalar, dim_>, dim_> {
  static constexpr size_t dim = dim_;
  using DataType = Eigen::Vector<Scalar, dim>;
  using BaseType = ManifoldBase_<Euclidean_, DataType, dim_>;
  using BaseType::ManifoldBase_;

  Euclidean_() : BaseType(DataType::Zero()) {}

  // in place!
  template <typename Derived>
  inline void perturb(const Eigen::MatrixBase<Derived>& dsp) {
    this->data += dsp;
  }

  inline auto compare(const Euclidean_& other) const {
    return this->data - other.data;
  }

  inline static const std::string name = "eucl";
};

struct ManifoldSO3
    : public ManifoldBase_<ManifoldSO3, math::MatrixN_<Scalar, 3>, 3> {
  using DataType = math::MatrixN_<Scalar, 3>;
  using BaseType = ManifoldBase_<ManifoldSO3, DataType, 3>;
  using BaseType::ManifoldBase_;

  ManifoldSO3() : BaseType(DataType::Identity()) {}

  template <typename Derived>
  inline void perturb(const Eigen::MatrixBase<Derived>& dsp) {
    this->data *= math::expMapSO3<Scalar>(dsp);
    math::fixRotation(this->data);
  }

  inline auto compare(const ManifoldSO3& other) const {
    return math::logMapSO3<Scalar>(other.data.transpose() * this->data);
  }

  inline static const std::string name = "SO3";
};

}  // namespace manifold
}  // namespace flib

namespace flib {
namespace manifold {
extern template class ManifoldComp_<
    ManifoldSO3, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
    Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>;
}

}  // namespace flib