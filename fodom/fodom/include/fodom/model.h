#pragma once

// State transition models

#include <fodom/manifold.h>

namespace fodom {

namespace IMUOutput {
using StateType =
    ManifoldComp_<ManifoldSO3,    // Orientation
                  Euclidean_<3>,  // Position
                  Euclidean_<3>,  // Velocity
                  Euclidean_<3>,  // Bias Gyro
                  Euclidean_<3>,  // Bias Acc
                  Euclidean_<3>,  // Gravity Vector global frame (Should be S2?)
                  Euclidean_<3>,  // Acceleration
                  Euclidean_<3>   // Angular Velocity
                  >;
using InputType = Vector_<Scalar, 12>;

// u: random walk bias noise (gyro, acc)
//    gaussian noise on angular acceleration and jerk
StateType::TangentType vels(const StateType& X, const InputType& u);

StateType fwd(const StateType& X, const InputType& u, Scalar dt);

// set initial conditions??
void setGrav(StateType& X, const Vector3_<Scalar>& g);

}  // namespace IMUOutput

}  // namespace fodom
