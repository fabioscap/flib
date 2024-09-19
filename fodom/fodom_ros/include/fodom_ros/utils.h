#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "flib/math.h"
#include "fodom/common.h"
#include "fodom/utils.h"

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
using ROSStopWatch = fodom::StopWatch_<rclcpp::Time>;

geometry_msgs::msg::Pose eigenToPoseMsg(
    const flib::math::Isometry3_<fodom::Scalar>& isometry);

}  // namespace fodom_ros