#ifndef _LIDAR_CONTAINER_HPP_
#define _LIDAR_CONTAINER_HPP_
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
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/erase.hpp>

using namespace std;
namespace prop = boost::property_tree;

namespace container
{
class LiDARContainer
{
public:
  LiDARContainer();
  ~LiDARContainer();
  bool initParam();
  bool getFileName(const int &num, string &file_path, string &file_img_path);
  bool loadFilePcd(const string &file_path);
  bool loadFileCsv(const string &file_path);
  bool getPower();
  bool create2DImgXY(cv::Mat &result_img);
  bool createDepthImg(cv::Mat &depth_img);
  cv::Vec3d linerInterporation(cv::Mat &src, const cv::Point &pix);
  bool createDepthColorImg(cv::Mat &depth_img, cv::Mat &depth_label_img);
  bool createRefrectImg(cv::Mat &ref);
  bool createRefrectColorImg(cv::Mat &ref_img, cv::Mat &ref_label_img);
  bool labelingDepth(cv::Mat &src, vector<cv::Rect> &rects);
  bool distanceComp(const double &target, const double &comp_pix);
  bool create2DImgXZ(cv::Mat &result_img);
  bool run();
  bool run(string &file_path, cv::Mat &lidar_img, cv::Mat &lidar_info);
private:
  std::vector<std::vector<double>> file_value_;
  std::string pcd_path_, file_format_, image_path_;
  int range_;
  double scale_;
  double pixel_;
  int power_;
  double accept_;
  int angle_, angle_offset_;
  double lidar_height_;
  double correction_1_, correction_2_, correction_z_; //補正値
  int debug_;
};

}
#endif