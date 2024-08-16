#include "fodom_ros/utils.h"

namespace fodom {

template <>
double time_to_sec(const builtin_interfaces::msg::Time& time) {
  return time.sec + time.nanosec * 1e-9;  // why ROS2, why
}
template <>
double time_to_sec(const rclcpp::Time& time) {
  return time.seconds();
}
template <>
double time_to_sec(const rclcpp::Duration& duration) {
  return duration.seconds();
}

}  // namespace fodom