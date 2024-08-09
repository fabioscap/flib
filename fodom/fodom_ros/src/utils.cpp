#include "fodom_ros/utils.h"

namespace fodom_ros {
double time_to_double(const builtin_interfaces::msg::Time& time) {
  return time.sec + time.nanosec * 1e-9;  // why ROS2, why
}
}  // namespace fodom_ros