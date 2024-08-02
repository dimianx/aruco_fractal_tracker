#include "aruco_fractal_tracker/aruco_fractal_tracker_node.hpp"

#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace fractal_tracker
{
ArucoFractalTracker::ArucoFractalTracker(const rclcpp::NodeOptions &options)
  : Node("aruco_fractal_tracker", options)
{
  this->declare_parameter<std::string>("marker_configuration", "");
  this->declare_parameter<double>("marker_size", 0.0);
  this->declare_parameter<std::string>("cam_params_file", "");

  auto cam_params_file = this->get_parameter("cam_params_file").get_value<std::string>();
  aruco::CameraParameters cam_params;
  cam_params.readFromXMLFile(cam_params_file);
  if (!cam_params.isValid())
    throw std::invalid_argument("Invalid camera parameters!");

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_input_topic", 10, std::bind(&ArucoFractalTracker::imageCallback, this, std::placeholders::_1));

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_output_topic", 10);

  marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "poses_output_topic", 10);

  auto marker_configuration = this->get_parameter("marker_configuration").get_value<std::string>();
  auto marker_size = this->get_parameter("marker_size").get_value<double>();
  
  detector_.setConfiguration(marker_configuration);
  detector_.setParams(cam_params, marker_size);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void ArucoFractalTracker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat gray;

  try 
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  } 
  catch (cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

  if (detector_.detect(gray))
  {
    detector_.drawMarkers(cv_ptr->image);
    
    std::vector<aruco::Marker> markers = detector_.getMarkers();
    for (auto&& marker : markers)
      marker.draw(cv_ptr->image, cv::Scalar(255, 255, 255), 2);

    detector_.draw2d(cv_ptr->image);

    if (detector_.poseEstimation())
    {
      cv::Mat tvec = detector_.getTvec();
      cv::Mat rvec = detector_.getRvec();
      detector_.draw3d(cv_ptr->image);
      
      cv::Mat rmatrix;
      cv::Rodrigues(rvec, rmatrix);
      tf2::Matrix3x3 tf2_rot(rmatrix.at<double>(0, 0), rmatrix.at<double>(0, 1), rmatrix.at<double>(0, 2),
                             rmatrix.at<double>(1, 0), rmatrix.at<double>(1, 1), rmatrix.at<double>(1, 2),
                             rmatrix.at<double>(2, 0), rmatrix.at<double>(2, 1), rmatrix.at<double>(2, 2));
        
      tf2::Vector3 tf2_translation(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
      tf2::Transform tf2_transform(tf2_rot, tf2_translation);
      tf2::Quaternion quat;
      tf2_rot.getRotation(quat);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = msg->header.frame_id;
      pose.header.stamp = this->get_clock()->now();
      pose.pose.position.x = tf2_translation.x();
      pose.pose.position.y = tf2_translation.y();
      pose.pose.position.z = tf2_translation.z();
      pose.pose.orientation.x = quat.getX();
      pose.pose.orientation.y = quat.getY();
      pose.pose.orientation.z = quat.getZ();
      pose.pose.orientation.w = quat.getW();

      marker_pose_pub_->publish(pose);
        

      int base_line = 0;
      int font_face = cv::FONT_HERSHEY_PLAIN;
      double font_scale = 1;
      int thickness = 1;
      int line_height = cv::getTextSize("W", font_face, font_scale, thickness, &base_line).height + 5;

      cv::Point pos_text_pos(10, 10 + line_height);
      cv::putText(cv_ptr->image, "POSITION", pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, "POSITION", pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      pos_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("X=%.2f", tf2_translation.x()), pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("X=%.2f", tf2_translation.x()), pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      pos_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("Y=%.2f", tf2_translation.y()), pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("Y=%.2f", tf2_translation.y()), pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      pos_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("Z=%.2f", tf2_translation.z()), pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("Z=%.2f", tf2_translation.z()), pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

      cv::Point ori_text_pos(cv_ptr->image.cols - 150, 10 + line_height);
      cv::putText(cv_ptr->image, "ORIENTATION", ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, "ORIENTATION", ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      ori_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("X=%.2f", quat.getX()), ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("X=%.2f", quat.getX()), ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      ori_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("Y=%.2f", quat.getY()), ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("Y=%.2f", quat.getY()), ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      ori_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("Z=%.2f", quat.getZ()), ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("Z=%.2f", quat.getZ()), ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      ori_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("W=%.2f", quat.getW()), ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("W=%.2f", quat.getW()), ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->get_clock()->now();
      transform.header.frame_id = msg->header.frame_id;
      transform.child_frame_id = "marker_frame";
      transform.transform.translation.x = tf2_translation.x();
      transform.transform.translation.y = tf2_translation.y();
      transform.transform.translation.z = tf2_translation.z();
      transform.transform.rotation.x = quat.getX();
      transform.transform.rotation.y = quat.getY();
      transform.transform.rotation.z = quat.getZ();
      transform.transform.rotation.w = quat.getW();

      tf_broadcaster_->sendTransform(transform);
    }
  }
  else
  {
    cv::putText(cv_ptr->image, "NOT FOUND", cv::Point(20, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(cv_ptr->image, "NOT FOUND", cv::Point(20, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  try 
  {
    image_pub_->publish(*cv_ptr->toImageMsg());
  } 
  catch (cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }
}

} // namespace ssl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fractal_tracker::ArucoFractalTracker)