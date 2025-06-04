#include "fodom/utils.h"

#include "fodom/common.h"

namespace fodom {
template <>
double time_to_sec(
    const std::chrono::duration<long int, std::ratio<1, 1000000000>>& d) {
  return double(d.count()) * 1e-9;
}
}  // namespace fodom
