#include "fodom_ros/fodom_ros.h"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

#include "fodom_ros/utils.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace fodom_ros {
using namespace fodom::IMUOutput;

FodomROS::FodomROS(const rclcpp::NodeOptions& options)
    : Node("fodom_ros_node", options) {
  using namespace std::placeholders;

  this->declare_parameter<double>("frequency", 1.0);
  dt_ = 1.0 / this->get_parameter("frequency").as_double();

  timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_),
                                   std::bind(&FodomROS::timer_callback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&FodomROS::imu_callback, this, _1));
}

void FodomROS::timer_callback() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = rclcpp::Node::now();
  t.header.frame_id = "world";
  t.child_frame_id = "robot";

  // auto p = X.get<1>().data();
  // t.transform.translation.x = p.x();
  // t.transform.translation.y = p.y();
  // t.transform.translation.z = p.z();
  tf_broadcaster_->sendTransform(t);

  // X = fwd(X, InputType::Zero(), dt_);

  RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f", 0, 0, 0, t.header.stamp);
}

void FodomROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "messaggio IMU");

  double time_double = time_to_double(msg->header.stamp);

  fodom::IMUMeasurement<double> m;
  m.stamp = time_double;
  m.lin_acc << msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z;
  m.ang_vel << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;
  X.process_IMU(m);
  RCLCPP_INFO(this->get_logger(), "%lf", time_double);
}

}  // namespace fodom_ros

RCLCPP_COMPONENTS_REGISTER_NODE(fodom_ros::FodomROS)