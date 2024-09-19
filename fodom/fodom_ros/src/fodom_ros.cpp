#include "fodom_ros/fodom_ros.h"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

#include "fodom/utils.h"
#include "fodom_ros/utils.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace fodom_ros {

FodomROS::FodomROS(const rclcpp::NodeOptions& options)
    : Node("fodom_ros_node", options) {
  using namespace std::placeholders;

  this->declare_parameter<double>("frequency", 1.0);
  dt_ = 1.0 / this->get_parameter("frequency").as_double();

  timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_),
                                   std::bind(&FodomROS::timer_callback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "pose_with_covariance", 10);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 1000, std::bind(&FodomROS::imu_callback, this, _1));

  ROSStopWatch sw([this]() { return this->now(); });

  robot_.sw_ = std::make_shared<ROSStopWatch>(sw);
  robot_.log_function_ = [this](const std::string& message) {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  };

  // robot_.state().g() << 0.0, 0.0, -0.01;
}
void FodomROS::timer_callback() {
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  auto now = rclcpp::Node::now();
  auto p = robot_.state().p();
  auto R = robot_.state().R();
  Eigen::Quaterniond quaternion(R);

  t.header.stamp = now;
  t.header.frame_id = "world";
  t.child_frame_id = "robot";

  t.transform.translation.x = p.x();
  t.transform.translation.y = p.y();
  t.transform.translation.z = p.z();
  t.transform.rotation.x = quaternion.x();
  t.transform.rotation.y = quaternion.y();
  t.transform.rotation.z = quaternion.z();
  t.transform.rotation.w = quaternion.w();
  tf_broadcaster_->sendTransform(t);

  msg.header.stamp = now;
  msg.header.frame_id = "world";

  msg.pose.pose.position.x = p.x();
  msg.pose.pose.position.y = p.y();
  msg.pose.pose.position.z = p.z();

  msg.pose.pose.orientation.x = quaternion.x();
  msg.pose.pose.orientation.y = quaternion.y();
  msg.pose.pose.orientation.z = quaternion.z();
  msg.pose.pose.orientation.w = quaternion.w();

  Eigen::Matrix<double, 6, 6> tmp = robot_.S_.block<6, 6>(0, 0);
  tmp.block<3, 3>(0, 0).swap(tmp.block<3, 3>(3, 3));
  tmp.block<3, 3>(3, 0).swap(tmp.block<3, 3>(0, 3));
  for (size_t i = 0; i < 6; ++i)
    for (size_t j = 0; j < 6; ++j) msg.pose.covariance[i * 6 + j] = tmp(i, j);

  pose_publisher_->publish(msg);

  // RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f", 0, 0, 0,
  // t.header.stamp);

  // robot_.propagate(decltype(robot_)::InputType::Zero(), dt_);
}

void FodomROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "processing IMU msg...");

  double time_double = time_to_sec(msg->header.stamp);

  fodom::IMUMeasurement<double> m;
  m.stamp = time_double;
  m.lin_acc << msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z;
  m.ang_vel << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;
  robot_.process_IMU(m);
  RCLCPP_INFO(this->get_logger(), "...done");
}

}  // namespace fodom_ros

RCLCPP_COMPONENTS_REGISTER_NODE(fodom_ros::FodomROS)