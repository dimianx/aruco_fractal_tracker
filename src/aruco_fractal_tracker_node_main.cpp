#include "aruco_fractal_tracker/aruco_fractal_tracker_node.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<fractal_tracker::ArucoFractalTracker>(options));
  rclcpp::shutdown();

  return 0;
}