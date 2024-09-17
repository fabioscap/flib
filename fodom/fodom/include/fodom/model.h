#pragma once

// State transition models

#include "flib/manifold.h"

namespace fodom {

using namespace flib::manifold;

struct IMUOutput : public ManifoldComp_<Euclidean_<3>> {};

}  // namespace fodom
