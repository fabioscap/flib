#include "fodom/utils.h"

#include "fodom/common.h"

namespace fodom {
template <>
double time_to_sec(
    const std::chrono::duration<long int, std::ratio<1, 1000000000>>& d) {
  return double(d.count()) * 1e-9;
}
}  // namespace fodom

#include "flib/manifold.h"
#include "flib/unscented.h"

namespace flib {
namespace manifold {
template class ManifoldComp_<ManifoldSO3, Euclidean_<3>, Euclidean_<3>,
                             Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
                             Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>;
}

namespace unscented {
using namespace manifold;
template class UnscentedTransform<ManifoldComp_<
    ManifoldSO3, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>,
    Euclidean_<3>, Euclidean_<3>, Euclidean_<3>, Euclidean_<3>>>;
}  // namespace unscented

}  // namespace flib
