#include "fodom_ros/viz_logger.h"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace fodom_ros {
using namespace fodom::IMUOutput;

VizLogger::VizLogger(const rclcpp::NodeOptions& options)
    : Node("viz_logger_node", options) {
  this->declare_parameter<double>("frequency", 1.0);
  dt_ = 1.0 / this->get_parameter("frequency").as_double();
  RCLCPP_INFO(this->get_logger(), "%f", dt_);
  pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_),
                                   std::bind(&VizLogger::timer_callback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  setGrav(X, {0, 0, 0.1});
}

void VizLogger::timer_callback() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = rclcpp::Node::now();
  t.header.frame_id = "world";
  t.child_frame_id = "robot";

  auto p = X.get<1>().data();
  t.transform.translation.x = p.x();
  t.transform.translation.y = p.y();
  t.transform.translation.z = p.z();

  tf_broadcaster_->sendTransform(t);

  X = fwd(X, InputType::Zero(), dt_);

  RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f", p.x(), p.y(), p.z(),
              t.header.stamp);
}
}  // namespace fodom_ros

RCLCPP_COMPONENTS_REGISTER_NODE(fodom_ros::VizLogger)