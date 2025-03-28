#pragma once

#include <iostream>
namespace flib {
namespace _private {
template <typename... Types>
struct ManifoldCompContainer;

template <typename MBase>
struct ManifoldCompContainer<MBase> {
  static constexpr size_t dim = MBase::dim;
  using Scalar = typename MBase::Scalar;

  MBase data;

  template <typename Derived>
  inline void perturb(const Eigen::MatrixBase<Derived>& dsp) {
    data.perturb(dsp);
  }

  template <typename Derived>
  inline void compare(const ManifoldCompContainer<MBase>& other,
                      Eigen::MatrixBase<Derived>& out) const {
    out = data.compare(other.data);
  }
};

template <typename FirstType, typename... Rest>
struct ManifoldCompContainer<FirstType, Rest...> {
  using ThisType = ManifoldCompContainer<FirstType, Rest...>;
  using Scalar = typename FirstType::Scalar;
  using RestType = ManifoldCompContainer<Rest...>;

  FirstType data;
  RestType rest;

  static constexpr size_t dim = FirstType::dim + RestType::dim;

  template <typename Derived>
  inline void perturb(const Eigen::MatrixBase<Derived>& dsp) {
    this->data.perturb(dsp.template head<FirstType::dim>());
    rest.perturb(dsp.template tail<RestType::dim>());
  }

  // unsure about this but comparison with tuple implementation shows no
  // difference (in speed)
  template <typename Derived>
  inline void compare(const ThisType& other,
                      Eigen::MatrixBase<Derived>& out) const {
    out.template head<FirstType::dim>() = data.compare(other.data);
    auto drest = out.template tail<RestType::dim>();
    rest.compare(other.rest, drest);
  }
};

template <typename... Types>
std::ostream& operator<<(std::ostream& os,
                         const _private::ManifoldCompContainer<Types...>& t) {
  return os << t.data << "\n" << t.rest;
}
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const _private::ManifoldCompContainer<T>& t) {
  return os << t.data;
}

}  // namespace _private
}  // namespace flib