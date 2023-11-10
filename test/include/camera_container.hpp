#ifndef _CAMERA_CONTAINER_HPP_
#define _CAMERA_CONTAINER_HPP_
/* c++ */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

/* boost(iniファイル読み込み) */
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

using namespace std;
namespace prop = boost::property_tree;

namespace container
{
class CameraContainer
{
public:
  CameraContainer();
  ~CameraContainer();
  bool initParam();
  bool changeHsv(const cv::Mat &src, cv::Mat &hsv);
  void colorDetector(const cv::Mat &src, cv::Mat &result, const vector<int> &param);
  // bool greenDetector(const cv::Mat &src, cv::Mat &result_green);
  // bool blueDetector(const cv::Mat &src, cv::Mat &result_blue);
  bool labeling(const cv::Mat &binary, vector<cv::Rect> &rects, double &area_per);
  // bool labeling(const cv::Mat &result_green, const cv::Mat &result_blue,
  //               vector<cv::Rect> &green_rect, vector<cv::Rect> &blue_rect);
  bool selectOneRectangle(vector<cv::Rect> &rects);
  void containerPose(cv::Mat &src, const cv::Mat &img, const cv::Rect &rect);
  bool run(const string &file_name, cv::Mat &camera_img, cv::Rect &blue_rect);
  bool run(const string &file_name);
  cv::Point2d rel_pose_ = cv::Point2d(0, 0);
private:
  /*** メンバ関数 ***/
  std::vector<int> green_hsv_param_;
  std::vector<int> blue_hsv_param_;
  int h_size_min1_, w_size_min1_, h_size_max1_, w_size_max1_;
  int h_size_min2_, w_size_min2_, h_size_max2_, w_size_max2_;
  double rate_min_, rate_max_, g_area_per_, b_area_per_;
  double angle_deg_, angle_offset_;
  int height_line_;
  int debug_;
};

}
#endif