#include <chrono>

#include "fodom/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace fodom {

template <>
double time_to_sec(const builtin_interfaces::msg::Time& time);
template <>
double time_to_sec(const rclcpp::Time& time);
template <>
double time_to_sec(const rclcpp::Duration& duration);
}  // namespace fodom

namespace fodom_ros {
using fodom::time_to_sec;

// ros time requires a node handle, you pass it to the constructor as such:
// ROSStopWatch([node_ptr]() { return node_ptr->now(); });
using ROSStopWatch = fodom::SimpleStopWatch_<rclcpp::Time, rclcpp::Duration>;

}  // namespace fodom_ros