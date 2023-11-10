#ifndef WAYPOINT_TOOL_HPP
#define WAYPOINT_TOOL_HPP

#include <opencv2/opencv.hpp>

class WayPointTool
{
public:
  WayPointTool();
  ~WayPointTool();
private:
  void onMoveMouse();
  void makeLocalMap(const cv::Mat &global_map, cv::Mat &local_map);
  void onMoveMouse();
};

#endif