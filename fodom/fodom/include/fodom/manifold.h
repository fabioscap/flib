#pragma once

#include <iostream>

#include "fodom/common.h"

namespace fodom {

size_t c_cnt{0};

constexpr bool log = false;
template <int dim>
using TangentType_ = Vector_<Scalar, dim>;
template <typename Derived>
using TangentTypeIn_ = Eigen::MatrixBase<Derived>;

template <typename DataType_, int dim_>
struct ManifoldBase_ {
  static constexpr int dim = dim_;
  using ThisType = ManifoldBase_<DataType_, dim_>;
  using DataType = DataType_;
  using TangentType = TangentType_<dim>;

  ManifoldBase_() = default;

  template <typename T>
  ManifoldBase_(T&& d) noexcept : data(std::forward<T>(d)) {}

  // define these two methods with the correct signature,
  // these two are just here to provide compile time
  // errors if your forget to define them.
  template <typename D>
  inline ThisType boxplus(const TangentTypeIn_<D>&) const {
    static_assert(sizeof(DataType_) == 0, "boxplus not defined");
  }
  TangentType boxminus(const ThisType&) const;

  DataType data;
};

// specialization for Euclidean domains
template <int dim>
struct Euclidean_ : public ManifoldBase_<TangentType_<dim>, dim> {
  using BaseType = ManifoldBase_<TangentType_<dim>, dim>;
  using DataType = typename BaseType::DataType;
  using TangentType = typename BaseType::TangentType;

  Euclidean_() {}
  template <typename T>
  Euclidean_(T&& data) noexcept : BaseType(std::forward<T>(data)) {
    // if constexpr (log) std::cout << "data ctor\n";
  }

  Euclidean_(const Euclidean_& other) {
    // if constexpr (log) std::cout << "cp";
  }

  Euclidean_(Euclidean_&& other) noexcept {
    // if constexpr (log) std::cout << "mv";
  }

  template <typename D>
  inline auto boxplus(const TangentTypeIn_<D>& dsp) const {
    // if constexpr (log) std::cout << "bplus eucl\n";
    return (this->data + dsp);
  }

  TangentType boxminus(const Euclidean_& to) const;

  inline static const std::string name = "e" + std::to_string(dim);
};

template <typename... Manifolds>
struct ManifoldComp_;

template <typename Manifold>
struct ManifoldComp_<Manifold> {
  static constexpr int dim = Manifold::dim;
  using ThisType = ManifoldComp_<Manifold>;
  using TangentType = typename Manifold::TangentType;

  ManifoldComp_() = default;
  template <typename T>
  ManifoldComp_(T&& m) noexcept : manifold_(std::forward<T>(m)) {
    // if constexpr (log) std::cout << "comp ctor " << dim << "\n";
  }

  template <typename D>
  inline ThisType boxplus(const Eigen::MatrixBase<D>& dsp) const {
    // if constexpr (log) std::cout << "bplus comp end\n";
    return ThisType{manifold_.boxplus(dsp)};
  }
  TangentType boxminus(const ThisType& to) const;

  template <size_t N>
  const Manifold& get() const;

  template <size_t N>
  Manifold& get();

  Manifold manifold_;
};

template <typename Manifold, typename... Rest>
struct ManifoldComp_<Manifold, Rest...> {
  static constexpr int dim = Manifold::dim + ManifoldComp_<Rest...>::dim;
  using ThisType = ManifoldComp_<Manifold, Rest...>;
  using TangentType = TangentType_<dim>;

  ManifoldComp_() = default;
  template <typename M, typename R>
  ManifoldComp_(M&& m, R&& r) noexcept
      : manifold_(std::forward<M>(m)), rest_(std::forward<R>(r)) {
    // if constexpr (log) std::cout << "comp ctor " << dim << "\n";
  }

  template <typename D>
  inline ThisType boxplus(const TangentTypeIn_<D>& dsp) const {
    return ThisType{
        std::move(manifold_.boxplus(dsp.template head<Manifold::dim>())),
        std::move(rest_.boxplus(dsp.template tail<dim - Manifold::dim>()))};
  }
  TangentType boxminus(const ThisType& to) const;

  template <size_t N>
  auto& get();

  template <size_t N>
  const auto& get() const;

  inline const ManifoldComp_<Rest...>& rest() const { return rest_; }

  Manifold manifold_;
  ManifoldComp_<Rest...> rest_;
};

template <typename DataType, int dim>
std::ostream& operator<<(std::ostream&, const ManifoldBase_<DataType, dim>&);
template <typename Manifold, typename... Rest>
std::ostream& operator<<(std::ostream&,
                         const ManifoldComp_<Manifold, Rest...>&);
template <typename Manifold>
std::ostream& operator<<(std::ostream&, const ManifoldComp_<Manifold>&);

// Implementations
struct ManifoldSO3 : public ManifoldBase_<MatrixN_<Scalar, 3>, 3> {
  using BaseType = ManifoldBase_<MatrixN_<Scalar, 3>, 3>;
  ManifoldSO3();
  ManifoldSO3(const DataType& data);
  ManifoldSO3 boxplus(const TangentType& dsp) const;
  TangentType boxminus(const ManifoldSO3& to) const;
};

struct ManifoldSO3Q : public ManifoldBase_<MatrixN_<Scalar, 3>, 3> {
  using BaseType = ManifoldBase_<MatrixN_<Scalar, 3>, 3>;
  ManifoldSO3Q();
  ManifoldSO3Q(const DataType& data);
  ManifoldSO3Q boxplus(const TangentType& dsp) const;
  TangentType boxminus(const ManifoldSO3Q& to) const;
};

template <typename DataType_, int dim_>
typename ManifoldBase_<DataType_, dim_>::TangentType
ManifoldBase_<DataType_, dim_>::boxminus(const ThisType&) const {
  static_assert(sizeof(DataType_) == 0, "boxminus not defined");
}

template <int dim>
typename Euclidean_<dim>::TangentType Euclidean_<dim>::boxminus(
    const Euclidean_<dim>& to) const {
  return this->data - to.data;
}

template <typename Manifold>
template <size_t N>
const Manifold& ManifoldComp_<Manifold>::get() const {
  static_assert(N == 0, "Index out of range");
  return manifold_;
}

template <typename Manifold>
template <size_t N>
Manifold& ManifoldComp_<Manifold>::get() {
  static_assert(N == 0, "Index out of range");
  return manifold_;
}

template <typename Manifold>
typename ManifoldComp_<Manifold>::TangentType ManifoldComp_<Manifold>::boxminus(
    const ThisType& to) const {
  return manifold_.boxminus(to.get<0>());
}

template <typename Manifold, typename... Rest>
typename ManifoldComp_<Manifold, Rest...>::TangentType
ManifoldComp_<Manifold, Rest...>::boxminus(const ThisType& to) const {
  ManifoldComp_<Manifold, Rest...>::TangentType out;
  // the first
  out.head(manifold_.dim) = manifold_.boxminus(to.get<0>());
  // the rest
  out.tail(dim - manifold_.dim) = rest_.boxminus(to.rest_);
  return out;
}

template <typename Manifold, typename... Rest>
template <size_t N>
auto& ManifoldComp_<Manifold, Rest...>::get() {
  if constexpr (N == 0)
    return manifold_;
  else
    return rest_.template get<N - 1>();
}

template <typename Manifold, typename... Rest>
template <size_t N>
const auto& ManifoldComp_<Manifold, Rest...>::get() const {
  if constexpr (N == 0)
    return manifold_;
  else
    return rest_.template get<N - 1>();
}

template <typename DataType, int dim>
std::ostream& operator<<(std::ostream& os,
                         const ManifoldBase_<DataType, dim>& m) {
  os << m.data;
  return os;
}

template <typename Manifold, typename... Rest>
std::ostream& operator<<(std::ostream& os,
                         const ManifoldComp_<Manifold, Rest...>& m) {
  os << m.template get<0>();
  os << "\n";

  os << m.rest();

  return os;
}

template <typename Manifold>
std::ostream& operator<<(std::ostream& os, const ManifoldComp_<Manifold>& m) {
  os << m.template get<0>();
  return os;
}
}  // namespace fodom