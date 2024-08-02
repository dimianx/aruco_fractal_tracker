#ifndef ARUCO_FRACTAL_TRACKER__ARUCO_FRACTAL_TRACKER_NODE_HPP_
#define ARUCO_FRACTAL_TRACKER__ARUCO_FRACTAL_TRACKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <aruco/fractaldetector.h>

#include <memory>

namespace fractal_tracker
{
class ArucoFractalTracker : public rclcpp::Node
{
public:
  explicit ArucoFractalTracker(const rclcpp::NodeOptions& options);

private:  
  aruco::FractalDetector detector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

}; // class ArucoFractalTracker
}  // namespace fractal_tracker
#endif  // ARUCO_FRACTAL_TRACKER__ARUCO_FRACTAL_TRACKER_NODE_HPP_
