#ifndef MAP_MAKE_HPP
#define MAP_MAKE_HPP

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "ros2_rs_miyauchi_interfaces/msg/way_point.hpp"

using namespace std;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using ros2_rs_miyauchi_interfaces::msg::WayPoint;

namespace monitor2023
{
class MainMapCreate
{
public:
  MainMapCreate();
  ~MainMapCreate();

private:
  /** 関数 **/
  void setMapImg(cv::Mat img);
  void setGridSize(const double size);
  void setMapViewSize(const double size);
  void setLocation(const Pose pose);
  void setWaypoint(const Waypoint waypoint);
  void setWaypoints();
  void setTarget(const Odometry odom);
  bool getUpdateState();
  cv::Mat getTrimImg();
  /** 変数 **/
  cv::Mat map_img_;
  double grid_size_;
  double map_view_size_;
  
}
}

#endif //MAP_MAKE_HPP