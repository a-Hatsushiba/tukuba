#ifndef MAP_MAKE_HPP
#define MAP_MAKE_HPP

#include <opencv2/opencv.hpp>
#include <ryusei/common/defs.hpp>
#include <ryusei/navi/waypoint_manager.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include "ros2_rs_miyauchi_interfaces/msg/way_point.hpp"

using namespace std;
using namespace project_ryusei;
// using geometry_msgs::msg::Pose;
// using nav_msgs::msg::Odometry;
// using ros2_rs_miyauchi_interfaces::msg::WayPoint;

namespace monitor2023
{
class MainMapCreate
{
public:
  MainMapCreate();
  ~MainMapCreate();
  /** 関数 **/
  static std::string getHomeDir();
  void setMapImg(cv::Mat img);
  void setGridSize(const double size);
  void setMapViewSize(const double size);
  // void setLocation(const Pose pose);
  // void setWaypoint(const Waypoint waypoint);
  void setWaypoints(const string file_name);
  // void setTarget(const Odometry odom);
  bool getUpdateState();
  cv::Mat getTrimImg();
  cv::Mat setLocation(int key);

private:
  /** 変数 **/
  cv::Mat global_map_;
  cv::Point global_map_center_;
  double grid_size_;
  double map_view_size_;
  // Pose location_;
  // Waypoint waypoint_;
  vector<WayPoint> way_points_;
  // Odometry target_odom_;
  cv::Point2d initial_pose_;
};
}

#endif //MAP_MAKE_HPP