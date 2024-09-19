
#include "fodom_ros/synth_imu_publisher.h"

#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "fodom_ros/utils.h"
#include "synthetic/synthetic.h"

using namespace std::chrono_literals;

namespace fodom_ros {

SynthImuPublisher::SynthImuPublisher(const rclcpp::NodeOptions& options)
    : Node("synth_imu_publisher_node", options) {
  using namespace std::placeholders;

  this->declare_parameter<double>("frequency", 10.0);
  freq_ = this->get_parameter("frequency").as_double();
  // traj_.generateData(freq_, data_);

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / freq_),
      std::bind(&SynthImuPublisher::timer_callback, this));
}
void SynthImuPublisher::timer_callback() {
  using namespace flib::math;
  using namespace fodom;
  sensor_msgs::msg::Imu msg;
  double sample_time{0};
  rclcpp::Time nowT = now();
  if (msg_cnt < 1) {
    ts = nowT;
    sample_time = 0;
  } else {
    sample_time = std::fmod(time_to_sec(nowT - ts), traj_.T());
  }
  IMUMeasurement<Scalar> meas;
  Isometry3_<Scalar> pose;
  Vector3_<Scalar> vel;
  traj_.getPoseMeasurement(sample_time, pose, vel, meas);

  msg.header.stamp = nowT;
  msg.linear_acceleration.x = meas.lin_acc.x();
  msg.linear_acceleration.y = meas.lin_acc.y();
  msg.linear_acceleration.z = meas.lin_acc.z();

  msg.angular_velocity.x = meas.ang_vel.x();
  msg.angular_velocity.y = meas.ang_vel.y();
  msg.angular_velocity.z = meas.ang_vel.z();

  imu_pub_->publish(msg);

  ++msg_cnt;

  geometry_msgs::msg::PoseStamped pose_msg;

  pose_msg.pose = eigenToPoseMsg(pose);
  pose_msg.header.stamp = nowT;
  pose_msg.header.frame_id = "world";

  path_msg_.header.stamp = nowT;
  path_msg_.header.frame_id = "world";

  path_msg_.poses.push_back(pose_msg);
  if (path_msg_.poses.size() > 100) {
    // sorry
    path_msg_.poses.erase(
        path_msg_.poses.begin(),
        path_msg_.poses.begin() + (path_msg_.poses.size() - 100));
  }
  path_pub_->publish(path_msg_);
}

}  // namespace fodom_ros

RCLCPP_COMPONENTS_REGISTER_NODE(fodom_ros::SynthImuPublisher)