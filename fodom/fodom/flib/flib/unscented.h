#pragma once

#include <Eigen/Dense>

#include "common.h"

namespace flib {
namespace unscented {

template <typename StateType>
class UnscentedTransform {
 public:
  enum class WeightScheme { UKF, HB };
  struct SigmaPoints {
    std::vector<StateType> points;
    Scalar wm0, wc0, wmi, wci;
  };

  using CovType = Eigen::Matrix<Scalar, StateType::dim, StateType::dim>;

  void toMeanCov(const SigmaPoints& spoints, StateType& mean,
                 CovType& cov) const;

  void toUnscented(const StateType& mean, const CovType& cov,
                   SigmaPoints& spoints) const;

  // protected:
  // mysterious parameters
  Scalar beta_{2.0f};
  Scalar alpha_{1e-3};
  Scalar k_{0};

  WeightScheme weight_scheme_ = WeightScheme::HB;

  Scalar cov_regularizer_ = 1e-100;

 private:
  void compute_weights_scaling(SigmaPoints& spoints, Scalar& cov_scaling) const;

  void compute_sigma_points(const StateType& mean, const CovType& L_scaled,
                            SigmaPoints& spoints) const;
  void mean_from_sigma_points(const SigmaPoints& spoints, StateType& mean,
                              const size_t n_iters = 1) const;
};

template <typename StateType>
void propagate(StateType& mean,
               typename UnscentedTransform<StateType>::CovType& cov,
               // void (*f)(StateType&)
               std::function<void(StateType&)> f) {
  UnscentedTransform<StateType> ut;
  typename UnscentedTransform<StateType>::SigmaPoints spoints;

  ut.toUnscented(mean, cov, spoints);

  std::for_each(spoints.points.begin(), spoints.points.end(),
                [](StateType& X) { std::cout << X << "\n---\n"; });
  std::for_each(spoints.points.begin(), spoints.points.end(), f);
  std::for_each(spoints.points.begin(), spoints.points.end(),
                [](StateType& X) { std::cout << X << "\n---\n"; });
  ut.toMeanCov(spoints, mean, cov);
}

}  // namespace unscented

}  // namespace flib

// implementation
namespace flib {
namespace unscented {

// utility functions
template <typename StateType>
void UnscentedTransform<StateType>::compute_weights_scaling(
    SigmaPoints& spoints, Scalar& cov_scaling) const {
  constexpr int state_dim = StateType::dim;

  Scalar& wm0 = spoints.wm0;
  Scalar& wc0 = spoints.wc0;
  Scalar& wmi = spoints.wmi;
  Scalar& wci = spoints.wci;

  if (weight_scheme_ == WeightScheme::UKF) {
    const Scalar lambda = alpha_ * alpha_ * (state_dim + k_) - state_dim;

    // weights 0
    wm0 = lambda / (state_dim + lambda);
    wc0 = wm0 + (1 - alpha_ * alpha_ + beta_);
    // weights i
    wmi = 0.5 / Scalar(state_dim + lambda);
    wci = 0.5 / Scalar(state_dim + lambda);

    cov_scaling = (state_dim + lambda);
  } else if (weight_scheme_ == WeightScheme::HB) {
    /* hertzberg weights*/
    wm0 = 1.0 / (2 * state_dim + 1);
    wmi = wm0;
    wc0 = 0.5;
    wci = wc0;
    cov_scaling = 1.0;
  } else {
    throw std::runtime_error(
        "UnscentedTransform::compute_weights_scaling| unknown weight scheme");
  }
}

template <typename StateType>
void UnscentedTransform<StateType>::compute_sigma_points(
    const StateType& mean, const CovType& L_scaled,
    SigmaPoints& spoints) const {
  constexpr int state_dim = StateType::dim;

  auto& points = spoints.points;

  points.resize(2 * state_dim + 1);
  points[0] = mean;
  for (size_t i = 0; i < state_dim; ++i) {
    // padre pio
    mean.boxplus(L_scaled.col(i), points[2 * i + 1]);
    mean.boxplus((-L_scaled.col(i)), points[2 * i + 2]);
  }
}

template <typename StateType>
void UnscentedTransform<StateType>::mean_from_sigma_points(
    const SigmaPoints& spoints, StateType& mean, const size_t n_iters) const {
  using TangentType = typename StateType::TangentType;
  const auto& points = spoints.points;
  const Scalar wm0 = spoints.wm0;
  const Scalar wmi = spoints.wmi;

  // compute chart point
  mean = points[0];

  for (size_t i = 0; i < n_iters; ++i) {
    // compute tangent mean displacement
    TangentType displacement = wm0 * mean.boxminus(points.at(0));
    for (size_t j = 1; j < points.size(); ++j) {
      const StateType& Xj = points.at(j);
      displacement += wmi * mean.boxminus(Xj);
    }
    // add it to the mean
    mean.boxplus_inplace(displacement);
  }
}

template <typename StateType>
void UnscentedTransform<StateType>::toMeanCov(const SigmaPoints& spoints,
                                              StateType& mean,
                                              CovType& cov) const {
  using TangentType = typename StateType::TangentType;

  const Scalar wc0 = spoints.wc0;
  const Scalar wci = spoints.wci;
  mean_from_sigma_points(spoints, mean, 0);
  const auto& points = spoints.points;
  // covariance lives in tangent space
  cov.setZero();
  TangentType err = mean.boxminus(points.at(0));
  cov += wc0 * err * err.transpose();

  for (size_t i = 1; i < points.size(); ++i) {
    err = mean.boxminus(points.at(i));
    cov += wci * err * err.transpose();
  }
}

template <typename StateType>
void UnscentedTransform<StateType>::toUnscented(const StateType& mean,
                                                const CovType& cov,
                                                SigmaPoints& spoints) const {
  Scalar cov_scaling;

  compute_weights_scaling(spoints, cov_scaling);
  // Perform Cholesky decomposition
  // TODO use selfAdjointView where it is possible
  Eigen::LLT<CovType> llt(cov_scaling *
                          (cov + CovType::Identity() * cov_regularizer_));

  if (!llt.info() == Eigen::Success)
    throw std::runtime_error(
        "UnscentedTransform::toUnscented| Cholesky decomposition failed");

  const CovType& L = llt.matrixL();

  compute_sigma_points(mean, L, spoints);
}

}  // namespace unscented
}  // namespace flib

namespace flib {

namespace unscented {
using namespace manifold;
extern template class UnscentedTransform<ManifoldComp_<
    ManifoldSO3, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
    Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>>;
}  // namespace unscented

}  // namespace flib