#pragma once

#include <iostream>

#include "fodom/common.h"

namespace fodom {

template <int dim>
using TangentType_ = Vector_<Scalar, dim>;

template <typename DataType_, int dim_>
class ManifoldBase_ {
 public:
  static constexpr int dim = dim_;
  using ThisType = ManifoldBase_<DataType_, dim_>;
  using DataType = DataType_;
  using TangentType = TangentType_<dim>;
  ManifoldBase_() = default;
  ManifoldBase_(const DataType& data) : data_(data) {}

  inline const DataType& data() const { return data_; }
  inline DataType& data() { return data_; }

  // redefine these two methods
  ThisType boxplus(const TangentType&) const;

  // convention for boxminus: compute chart around *this
  TangentType boxminus(const ThisType&) const;

 protected:
  DataType data_;
};

class ManifoldSO3 : public ManifoldBase_<MatrixN_<Scalar, 3>, 3> {
 public:
  using BaseType = ManifoldBase_<MatrixN_<Scalar, 3>, 3>;
  ManifoldSO3();
  ManifoldSO3(const DataType& data);
  ManifoldSO3 boxplus(const TangentType& dsp) const;
  TangentType boxminus(const ManifoldSO3& to) const;
};

class ManifoldSO2 : public ManifoldBase_<Scalar, 1> {
 public:
  using BaseType = ManifoldBase_<Scalar, 1>;
  ManifoldSO2();
  ManifoldSO2(const DataType& data);
  ManifoldSO2 boxplus(const TangentType& dsp) const;
  TangentType boxminus(const ManifoldSO2& to) const;
};

// specialization for Euclidean domains
template <int dim>
class Euclidean_ : public ManifoldBase_<TangentType_<dim>, dim> {
 public:
  using BaseType = ManifoldBase_<TangentType_<dim>, dim>;
  using DataType = typename BaseType::DataType;
  using TangentType = typename BaseType::TangentType;
  Euclidean_() : BaseType(DataType::Zero()) {}
  Euclidean_(const DataType& data) : BaseType(data) {}
  Euclidean_ boxplus(const TangentType& dsp) const;

  TangentType boxminus(const Euclidean_& to) const;
};

template <typename... Manifolds>
class ManifoldComp_;

template <typename Manifold>
class ManifoldComp_<Manifold> {
 public:
  static constexpr int dim = Manifold::dim;
  using ThisType = ManifoldComp_<Manifold>;
  using TangentType = typename Manifold::TangentType;

  ManifoldComp_() = default;

  ManifoldComp_(const Manifold& m);

  ThisType boxplus(const TangentType& dsp) const;

  // convention for boxminus: compute chart around *this
  TangentType boxminus(const ThisType& to) const;

  template <size_t N>
  const Manifold& get() const;

  template <size_t N>
  Manifold& get();

 protected:
  Manifold manifold_;
};

template <typename Manifold, typename... Rest>
class ManifoldComp_<Manifold, Rest...> {
 public:
  static constexpr int dim = Manifold::dim + ManifoldComp_<Rest...>::dim;
  using ThisType = ManifoldComp_<Manifold, Rest...>;
  using TangentType = TangentType_<dim>;

  ThisType boxplus(const TangentType& dsp) const;

  // convention for boxminus: compute chart around *this
  TangentType boxminus(const ThisType& to) const;

  template <size_t N>
  auto& get();

  template <size_t N>
  const auto& get() const;

  inline const ManifoldComp_<Rest...>& rest() const { return rest_; }

 protected:
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
template <typename DataType_, int dim_>
ManifoldBase_<DataType_, dim_> ManifoldBase_<DataType_, dim_>::boxplus(
    const TangentType&) const {
  throw std::runtime_error("boxplus not defined");
}

template <typename DataType_, int dim_>
typename ManifoldBase_<DataType_, dim_>::TangentType
ManifoldBase_<DataType_, dim_>::boxminus(const ThisType&) const {
  throw std::runtime_error("boxminus not defined");
}

template <typename Manifold>
ManifoldComp_<Manifold>::ManifoldComp_(const Manifold& m) : manifold_(m) {}

template <int dim>
Euclidean_<dim> Euclidean_<dim>::boxplus(const TangentType& dsp) const {
  return Euclidean_(this->data_ + dsp);
}
template <int dim>
typename Euclidean_<dim>::TangentType Euclidean_<dim>::boxminus(
    const Euclidean_<dim>& to) const {
  return to.data() - this->data_;
}

template <typename Manifold>
ManifoldComp_<Manifold> ManifoldComp_<Manifold>::boxplus(
    const TangentType& dsp) const {
  return ManifoldComp_<Manifold>(manifold_.boxplus(dsp));
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
ManifoldComp_<Manifold, Rest...> ManifoldComp_<Manifold, Rest...>::boxplus(
    const TangentType& dsp) const {
  ManifoldComp_<Manifold, Rest...> out;
  // the first
  out.get<0>() = manifold_.boxplus(dsp.head(manifold_.dim));
  // the rest
  out.rest_ = rest_.boxplus(dsp.tail(dim - manifold_.dim));
  return out;
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
  os << m.data();
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