#include <iostream>

#include "fodom/model.h"
using namespace fodom;
int main() {
  using ModelType = IMUInput;

  ModelType::StateType S;
  std::cout << S << "\n";
  std::cout << fwd<ModelType>(S, Vector_<Scalar, 6>::Ones(), 1) << "\n";
  return 0;
}