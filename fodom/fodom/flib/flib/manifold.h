#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <utility>

#include "common.h"
#include "manifold_impl.h"
#include "math.h"

namespace flib {
namespace manifold {

constexpr bool log = false;

template <typename Derived, typename DataType_, size_t dim_>
struct ManifoldBase_ {
  using DataType = DataType_;
  DataType data;

  static constexpr size_t dim = dim_;
  using TangentType = Eigen::Vector<double, dim>;

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

template <typename Derived, typename DataType, size_t dim>
std::ostream& operator<<(std::ostream& os,
                         const ManifoldBase_<Derived, DataType, dim>& t) {
  return os << t.data;
}

template <typename... Types>
struct ManifoldComp_
    : public ManifoldBase_<ManifoldComp_<Types...>,
                           _private::ManifoldCompContainer<Types...>,
                           _private::ManifoldCompContainer<Types...>::dim> {
  using ThisType = ManifoldComp_<Types...>;
  using TangentType = typename ManifoldBase_<
      ManifoldComp_<Types...>, _private::ManifoldCompContainer<Types...>,
      _private::ManifoldCompContainer<Types...>::dim>::TangentType;

  template <typename Derived>
  inline void perturb(const Eigen::MatrixBase<Derived>& dsp) {
    this->data.perturb(dsp);
  }

  inline auto compare(const ThisType& other) const {
    TangentType d;
    this->data.compare(other.data, d);

    return d;
  }
};

template <typename Scalar_, size_t dim_>
struct Euclidean_ : public ManifoldBase_<Euclidean_<Scalar_, dim_>,
                                         Eigen::Vector<Scalar_, dim_>, dim_> {
  static constexpr size_t dim = dim_;
  using Scalar = Scalar_;
  using DataType = Eigen::Vector<Scalar, dim>;
  using BaseType = ManifoldBase_<Euclidean_, DataType, dim_>;
  using BaseType::ManifoldBase_;

  Euclidean_() : BaseType(DataType::Zero()) {}

  template <typename Derived>
  inline void perturb(const Eigen::MatrixBase<Derived>& dsp) {
    this->data += dsp;
  }

  inline auto compare(const Euclidean_& other) const {
    return this->data - other.data;
  }

  inline static const std::string name = "eucl";
};

template <typename Scalar_>
struct SO3_
    : public ManifoldBase_<SO3_<Scalar_>, math::MatrixN_<Scalar_, 3>, 3> {
  using Scalar = Scalar_;
  using DataType = math::MatrixN_<Scalar, 3>;
  using BaseType = ManifoldBase_<SO3_, DataType, 3>;
  using BaseType::ManifoldBase_;

  SO3_() : BaseType(DataType::Identity()) {}

  template <typename Derived>
  inline void perturb(const Eigen::MatrixBase<Derived>& dsp) {
    this->data *= math::expMapSO3<Scalar>(dsp);
    math::fixRotation(this->data);
  }

  inline auto compare(const SO3_& other) const {
    return math::logMapSO3<Scalar>(other.data.transpose() * this->data);
  }

  inline static const std::string name = "SO3";
};

template <typename Scalar>
using ManifoldSO3_ = SO3_<Scalar>;

}  // namespace manifold
}  // namespace flib
