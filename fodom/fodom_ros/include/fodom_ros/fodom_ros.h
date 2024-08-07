#include "fodom/model.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace fodom_ros {

// Publish ROS2 messages to be visualized in RViz
class VizLogger : public rclcpp::Node {
 public:
  VizLogger(const rclcpp::NodeOptions& options);

 protected:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  fodom::IMUOutput::StateType X;

  double dt_ = 1.23;
};
}  // namespace fodom_ros