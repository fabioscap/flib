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

namespace fodom_ros {
geometry_msgs::msg::Pose eigenToPoseMsg(
    const flib::math::Isometry3_<fodom::Scalar>& isometry) {
  geometry_msgs::msg::Pose pose_msg;

  pose_msg.position.x = isometry.translation().x();
  pose_msg.position.y = isometry.translation().y();
  pose_msg.position.z = isometry.translation().z();

  // Set orientation
  Eigen::Quaterniond quat(isometry.rotation());
  pose_msg.orientation.x = quat.x();
  pose_msg.orientation.y = quat.y();
  pose_msg.orientation.z = quat.z();
  pose_msg.orientation.w = quat.w();

  return pose_msg;
}
}  // namespace fodom_ros