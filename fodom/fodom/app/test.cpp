#include <iostream>

#include "fodom/manifold.h"
#include "fodom/model.h"
#include "fodom/utils.h"

using namespace fodom;

ChronoStopWatch sw;
static constexpr size_t N = 1000;
IMUOutput::TangentType u = IMUOutput::TangentType::Ones();
template <typename T>
double benchmark(T& x) {
  sw.tic();
  for (size_t i = 0; i < N; ++i) {
    x.boxplus(u);
  }
  return sw.toc() / N;
}

int main() {
  ManifoldComp_<ManifoldSO3, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
                Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>
      manifold;
  ManifoldComp_<ManifoldSO3, Euclidean_<21>> manifold_opt;
  Euclidean_<IMUOutput::dim> eucl;
  ManifoldComp_<Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
                Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>
      eucl_comp;
  ManifoldComp_<ManifoldSO3Q, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
                Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>
      modelq;
  ManifoldComp_<ManifoldSO3Q, Euclidean_<21>> modelq_opt;
  ManifoldComp_<Euclidean_<3>, Euclidean_<21>> eucl3_21;

  // std::cout << "manifold took:       " << benchmark(manifold) << "
  // seconds\n";
  std::cout << "manifold_opt took:   " << benchmark(manifold_opt)
            << " seconds \n";

  std::cout << "euclidean took:      " << benchmark(eucl) << " seconds \n";
  std::cout << "euclidean_comp took: " << benchmark(eucl_comp) << " seconds \n";
  std::cout << "modelq took:         " << benchmark(modelq) << " seconds\n";
  std::cout << "modelq_opt took:     " << benchmark(modelq_opt)
            << " seconds \n";
  std::cout << "eucl3_21 took:     " << benchmark(eucl3_21) << " seconds \n";

  return 0;
}