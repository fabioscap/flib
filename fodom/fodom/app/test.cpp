#include <iostream>

#include "fodom/manifold.h"
#include "fodom/model.h"
#include "fodom/utils.h"

using namespace fodom;

int main() {
  static constexpr size_t N = 100000;

  ChronoStopWatch sw;

  IMUOutput::BaseType manifold;
  IMUOutput model;
  Euclidean_<IMUOutput::dim> eucl;
  ManifoldComp_<Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
                Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>
      eucl_comp;

  IMUOutput::TangentType u = IMUOutput::TangentType::Ones();

  sw.tic();
  for (size_t i = 0; i < N; ++i) {
    model.boxplus(u);
  }
  std::cout << "model took: " << sw.toc() << " seconds \n";

  sw.tic();
  for (size_t i = 0; i < N; ++i) {
    manifold.boxplus(u);
  }
  std::cout << "manifold took: " << sw.toc() << " seconds \n";

  sw.tic();
  for (size_t i = 0; i < N; ++i) {
    eucl.boxplus(u);
  }
  std::cout << "euclidean took: " << sw.toc() << " seconds \n";
  sw.tic();
  for (size_t i = 0; i < N; ++i) {
    eucl_comp.boxplus(u);
  }
  std::cout << "euclidean_comp took: " << sw.toc() << " seconds \n";

  return 0;
}