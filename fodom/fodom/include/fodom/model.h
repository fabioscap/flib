// State transition models

#include <fodom/common.h>
#include <fodom/manifold.h>

namespace fodom {

struct IMUInput {
  using StateType = ManifoldComp_<ManifoldSO3, Euclidean_<3>, Euclidean_<3>>;
  using InputType = Vector_<Scalar, 6>;  // angular velocity + acceleration

  // Xdot = vels(X,u)
  static inline StateType::TangentType vels(const StateType& X,
                                            const InputType& u) {
    StateType::TangentType v;

    v.head<3>() = u.head<3>();
    v.segment<3>(3) = X.get<2>().data();
    v.tail<3>() = u.tail<3>();

    return v;
  }
};

// Discrete state transition
template <typename ModelType_>
typename ModelType_::StateType fwd(const typename ModelType_::StateType& X,
                                   const typename ModelType_::InputType& u,
                                   Scalar dt) {
  return X.boxplus(ModelType_::vels(X, u) * dt);
}

template IMUInput::StateType fwd<IMUInput>(const IMUInput::StateType& X,
                                           const IMUInput::InputType& u,
                                           Scalar dt);

}  // namespace fodom
