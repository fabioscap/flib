#include <bit>

#include "fodom/odom.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace fodom_ros {

// Publish ROS2 messages to be visualized in RViz
class FodomROS : public rclcpp::Node {
 public:
  FodomROS(const rclcpp::NodeOptions& options);

 protected:
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  fodom::Odom robot_;

  double dt_ = 1.23;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::vector<fodom::Point<fodom::Scalar>> cloud_;
};
}  // namespace fodom_ros