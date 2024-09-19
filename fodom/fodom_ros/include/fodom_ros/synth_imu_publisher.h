#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "synthetic/synthetic.h"

namespace fodom_ros {

class SynthImuPublisher : public rclcpp::Node {
 public:
  SynthImuPublisher(const rclcpp::NodeOptions& options);

 protected:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;

  double freq_{1};

  synthetic::SE3EightTrajectory traj_{20};
  /*   std::vector<std::tuple<synthetic::IMUMeasurement<fodom::Scalar>,
                           flib::math::Vector3_<fodom::Scalar>,
                           flib::math::Isometry3_<fodom::Scalar>>>
        data_; */
  size_t msg_cnt{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Time ts;
};
}  // namespace fodom_ros