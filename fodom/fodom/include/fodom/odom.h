#pragma once

#include <functional>
#include <memory>

#include "fodom/common.h"
#include "fodom/model.h"
#include "fodom/utils.h"

namespace fodom {

class Odom {
  /*  public:
    using ModelType = IMUOutput;
    using InputType = ModelType::InputType;
    using CovType = MatrixN_<Scalar, ModelType::dim>;

    Odom(
        void (*f)(const std::string&) =
            [](const std::string& s) { std::cout << s << std::endl; },
        const std::shared_ptr<StopWatchBase>& sw =
            std::make_shared<ChronoStopWatch>());

    void process_IMU(const IMUMeasurement<Scalar>& meas);

    void propagate(const InputType& u, const Scalar dt);

    // protected:
    std::function<void(const std::string&)> log_function_;
    std::shared_ptr<StopWatchBase> sw_;

    Scalar stamp_ = 0.0;
    Scalar dt_min_ = 1e-3;
    Scalar dt_max_ = 1e-3;

    ModelType X_;
    CovType S_ = 0 * CovType::Identity();

    Scalar b_gyro_cov_ = 1e-4;  // (rad/s)^2
    Scalar b_accl_cov_ = 1e-4;  // (m/s^2)^2

    Scalar n_gyro_cov_ = 1e-4;  // (rad/s^2)^2
    Scalar n_accl_cov_ = 1e-4;  // (m/s^3)^2 */
};
}  // namespace fodom