
#include "fodom_ros/synth_imu_publisher.h"

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "fodom_ros/utils.h"
#include "synthetic/synthetic.h"

using namespace std::chrono_literals;

namespace fodom_ros {

SynthImuPublisher::SynthImuPublisher(const rclcpp::NodeOptions& options)
    : Node("synth_imu_publisher_node", options) {
  using namespace std::placeholders;
  RCLCPP_INFO(this->get_logger(), "ciaooo");

  this->declare_parameter<double>("frequency", 10.0);
  freq_ = this->get_parameter("frequency").as_double();
  // traj_.generateData(freq_, data_);

  this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / freq_),
      std::bind(&SynthImuPublisher::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "done ctor");
}
void SynthImuPublisher::timer_callback() {
  using namespace flib::math;
  using namespace fodom;
  RCLCPP_INFO(this->get_logger(), "timer cb");
  sensor_msgs::msg::Imu msg;
  double sample_time{0};
  rclcpp::Time nowT = now();
  if (msg_cnt < 1) {
    ts = nowT;
    sample_time = 0;
  } else {
    sample_time = time_to_sec(nowT - ts);
  }
  RCLCPP_INFO(this->get_logger(), ".timer cb");
  IMUMeasurement<Scalar> meas;
  Isometry3_<Scalar> pose;
  Vector3_<Scalar> vel;
  traj_.getPoseMeasurement(sample_time, pose, vel, meas);

  msg.header.stamp = nowT;
  msg.linear_acceleration.x = meas.lin_acc.x();
  msg.linear_acceleration.y = meas.lin_acc.y();
  msg.linear_acceleration.z = meas.lin_acc.z();
  RCLCPP_INFO(this->get_logger(), "..timer cb");
  imu_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "...timer cb");
  ++msg_cnt;
}

}  // namespace fodom_ros

RCLCPP_COMPONENTS_REGISTER_NODE(fodom_ros::SynthImuPublisher)