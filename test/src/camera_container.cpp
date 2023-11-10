#include "camera_container.hpp"
// #include "lidar_container.hpp"
/** c++ **/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <algorithm>

// #define height_line_ 10 //pixel

using namespace std;
namespace prop = boost::property_tree;

namespace container
{

CameraContainer::CameraContainer()
{
  initParam();
}


CameraContainer::~CameraContainer()
{

}


bool CameraContainer::initParam()
{
  // cout << __func__ << " start!!!" << endl;
  /*** iniファイルの読み込み ***/
  prop::ptree pt;
  prop::read_ini("../cfg/lidar_container.ini", pt);

  /*** 色パラメータ ***/
  int green_hue_min_, green_hue_max_, green_sat_min_, green_val_min_;
  int blue_hue_min_, blue_hue_max_, blue_sat_min_, blue_val_min_;
  if(auto v = pt.get_optional<int>("GreenParameter.HueMin")) green_hue_min_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.HueMax")) green_hue_max_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.SatMin")) green_sat_min_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.ValMin")) green_val_min_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.HueMin")) blue_hue_min_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.HueMax")) blue_hue_max_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.SatMin")) blue_sat_min_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.ValMin")) blue_val_min_ = v.get();
  if(auto v = pt.get_optional<int>("Parameter.Debug")) debug_ = v.get();
  green_hsv_param_ = {green_hue_min_, green_hue_max_, green_sat_min_, green_val_min_};
  blue_hsv_param_ = {blue_hue_min_, blue_hue_max_, blue_sat_min_, blue_val_min_};

  /*** 大きさパラメータ ***/
  if(auto v = pt.get_optional<double>("SizeParameter.HSizeMin1")) h_size_min1_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.WSizeMin1")) w_size_min1_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.HSizeMax1")) h_size_max1_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.WSizeMax1")) w_size_max1_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.HSizeMin2")) h_size_min2_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.WSizeMin2")) w_size_min2_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.HSizeMax2")) h_size_max2_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.WSizeMax2")) w_size_max2_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.RateMin")) rate_min_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.RateMax")) rate_max_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.GAreaPer")) g_area_per_ = v.get();
  if(auto v = pt.get_optional<double>("SizeParameter.BAreaPer")) b_area_per_ = v.get();

  /*** その他 ***/
  if(auto v = pt.get_optional<double>("Parameter.Angle")) angle_deg_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.AngleOffset")) angle_offset_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.HeightLine")) height_line_ = v.get();

  return true;
}


bool CameraContainer::changeHsv(const cv::Mat &src, cv::Mat &hsv)
{
  // cout << __func__ << " start!!!" << endl;
  /*** 画像をBGR->HSVに変換 ***/
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  /*** 明度(V)のヒストグラム平坦化 ***/
  vector<cv::Mat> planes(3);
  vector<cv::Mat> hsv_merge(3);
  //3つのチャンネル(H, S, V)に分離
  cv::split(hsv, planes);
  //Vのヒストグラム平坦化を行う
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
  // clahe->apply(planes[1], planes[1]);//これを消したほうがよく検出できる
  clahe->apply(planes[2], planes[2]);
  //分離したHSVのチャンネルを合成
  hsv_merge[0] = planes[0];
  hsv_merge[1] = planes[1];
  hsv_merge[2] = planes[2];
  cv::merge(hsv_merge, hsv);
}

/*** 特定色の抽出を行う(緑) ***///できたよ
void CameraContainer::colorDetector(const cv::Mat &src, cv::Mat &result, const vector<int> &param)
{
  static int count = 0;
  count++;
  // cout << __func__ << " start!!!" << endl;
  /*** 出力画像を0で初期化しておく ***/
  result = cv::Mat(src.size(), CV_8U, cv::Scalar(0));
  /*** 下半分の範囲でしきい値内である画素を抽出 ***/
  for(int rows = src.rows, y = rows * 1 / 2 ; y < rows; y++){
    /*** 行の先頭のポインタを取得 ***/
    const cv::Vec3b *ptr_src = src.ptr<cv::Vec3b>(y);
    uchar *ptr_result = result.ptr<uchar>(y);
    for(int x = 0, cols = src.cols; x < cols; x++){
      /*** コンテナを検出 ***/
      if((param[0] <= ptr_src[x][0] && ptr_src[x][0] <= param[1])
          && (param[2] <= ptr_src[x][1])
          && (param[3] <= ptr_src[x][2])){
            ptr_result[x] = 255;
      }
    }
  }
  if(count%2 == 1 || debug_){
    cv::imshow("detect", result);
  }
}


bool CameraContainer::labeling(const cv::Mat &binary, vector<cv::Rect> &rects, double &area_per)
{
  // cout << __func__ << " start!!!" << endl;
  cv::Mat labels, stats, centroids;
  int rows = binary.rows;
  int rows_start = rows * 1 / 2;
  int rows_range = rows - rows_start;
  /*** 緑コン4近傍でラベリング ***/
  int n = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 4);
  for(int i = 0; i < n; i++){
    /*** 最初の領域は画像の外枠なので無視 ***/
    if(i == 0) continue;
    /*** stats変数からラベル領域の矩形情報を取得 ***/
    int *param = stats.ptr<int>(i);
    int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
    int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
    int w = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
    int h = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
    int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
    /*** ラベル領域の保存 ***/
    /*** 新(条件追加&改良版) ***/
    if(y >= rows_start && y < rows_start + rows_range / 2){
      if(((h > h_size_min1_ && h < h_size_max1_) && (w > w_size_min1_ && w < w_size_max1_))
          && (rate_min_ < ((double)w / h) && ((double)w / h) < rate_max_) && (area >= h * w *area_per)){
        rects.push_back(cv::Rect(x, y, w, h));
      }
    }else if(y >= rows_start + rows_range / 2 && y < rows){
      if(((h > h_size_min2_ && h < h_size_max2_) && (w > w_size_min2_ && w < w_size_max2_))
          && (area >= h * w *area_per)){
        rects.push_back(cv::Rect(x, y, w, h));
      }
    }
  }
  return true;
}


/*** 矩形が複数個検出されてしまったときの処理 ***/
bool CameraContainer::selectOneRectangle(vector<cv::Rect> &rects)
{
  // cout << __func__ << " start!!!" << endl;
  /*** 床においてあるコンテナは他の検出矩形よりも下の位置にあるのではないかという過程に基づいたもの
   * (つまりyの値が最も大きいyをコンテナと認識) ***/
  /*** rectsの最後の要素を取り出せばOK ***/
  rects.erase(rects.begin(), rects.end() - 1);
  return true;
}

/** コンテナの位置を求める **/
void CameraContainer::containerPose(cv::Mat &src, const cv::Mat &binary, const cv::Rect &rect)
{
  /*** メディアンフィルタ適用(ノイズ消すため) ***/
  cv::medianBlur(binary, binary, 3);

  /** 箱の高さのピクセル数を出す **/
  bool first_flag = true;
  int h_dis, h_min, h_max;
  for(int y = 0; y < rect.width; y++){
    if(binary.at<uchar>(rect.y + y, rect.x + height_line_) != 0){
      if(first_flag){
        h_min = y;
        first_flag = false;
      }
      h_max = y;
    }
  }
  h_dis = h_max - h_min;
  // cout << "h_min: " << h_min << " h_max: " << h_max << " h_dis: " << h_dis << endl;
  // cout << h_min << "," << h_max << "," << h_dis << ",";
  /*** 描画 ***/
  cv::line(src, cv::Point(rect.x + height_line_, rect.y + h_min), cv::Point(rect.x + height_line_, rect.y + h_max), cv::Scalar(0, 0, 255), 1, 1);

  /*** h_disから相対座標での位置を求める ***/
  /* 矩形の中心のx座標(Mat) */
  int rect_center_x = rect.x + (rect.width / 2);
  // cout << "test center: " << rect_center_x << endl;
  if(rect_center_x < src.cols/2) angle_offset_ = abs(angle_offset_);
  else angle_offset_ = -abs(angle_offset_);
  double rel_pose_rad = ((src.cols/2 - rect_center_x) * ((angle_deg_/2 + angle_offset_) / 640)) * M_PI / 180;
  cout << "test deg: " << ((src.cols/2 - rect_center_x) * ((angle_deg_/2 + angle_offset_) / 640)) << endl; 
  cout << "test rad: " << rel_pose_rad << endl;
  // double pose_dis = 236.51 * pow(h_dis, -0.928);
  // cout << "test dis: " << pose_dis << endl;
  
  rel_pose_.x = 233.82 * pow(h_dis, -0.927); //データセットから求めた関数//ok
  rel_pose_.y = rel_pose_.x * tan(rel_pose_rad); //問題点あり
  // rel_pose_.y = sqrt(pose_dis * pose_dis - rel_pose_.x * rel_pose_.x);
  // rel_pose_.x = pose_dis * cos(rel_pose_rad);
  // rel_pose_.y = pose_dis * sin(rel_pose_rad);
  cout << "rel_pose_: " << rel_pose_ << endl; 
}


bool CameraContainer::run(const string &file_name, cv::Mat &camera_img, cv::Rect &blue_rect)
{
  // cout << __func__ << " start!!!" << endl;
  cout << "file name is " << file_name << endl;
  /*** 引数のファイルpathから画像を取得 ***/
  cv::Mat src = cv::imread(file_name, 1);
  if(src.empty()) {
    cout << "画像の読み込みに失敗しました" << endl;
    return false;
  }
  if(debug_) cv::imshow("src", src);
  /*** src画像をhsv画像に変換する ***/
  cv::Mat hsv;
  if(changeHsv(src, hsv)){
    /*** 色情報でコンテナを検出 ***/
    /*** ラベリングし、矩形の大きさと矩形の比率でコンテナ認識精度を上げる ***/
    cv::Mat green_binary_img, blue_binary_img;
    vector<cv::Rect> green_rects, blue_rects;
    colorDetector(hsv, blue_binary_img, blue_hsv_param_);
    labeling(blue_binary_img, blue_rects, b_area_per_);
    colorDetector(hsv, green_binary_img, green_hsv_param_);
    labeling(green_binary_img, green_rects, g_area_per_);
    if(!blue_rects.empty()){
      /*** 矩形が複数あった場合 ***/
      if(blue_rects.size() > 1){ //青コンテナが複数個
        // cornerDetection(src, blue_binary_img, blue_rects, pt);
        selectOneRectangle(blue_rects);
      }
      /*** 矩形描画 ***/
      for (auto &rect : blue_rects){
        rectangle(src, rect, cv::Scalar(255, 0, 0), 2);
      }
      blue_rect = blue_rects[0];
      // cout << "b_rect" <<blue_rects[0].height << endl;

      /*** 青コンテナの位置を計測 ***/
      containerPose(src, blue_binary_img, blue_rect);
    }
    if(!green_rects.empty()){
      /*** 矩形が複数あった場合 ***/
      if(green_rects.size() > 1){ //緑コンテナが複数個
      // cornerDetection(src, green_binary_img, green_rects, pt);
        selectOneRectangle(green_rects);
      }
      /*** 矩形描画 ***/
      for (auto &rect : green_rects){
        rectangle(src, rect, cv::Scalar(0, 255, 0), 2);
        // cv::line(src, cv::Point(rect.x + height_line_, rect.y), cv::Point(rect.x + height_line_, rect.y + rect.height), cv::Scalar(0, 0, 255), 1, 1);
      }
      // cout << "g_rect" << green_rects[0].height << endl;
    }
    /*** 画像表示 ***/
    // cv::resize(src, src, cv::Size(), 0.5, 0.5);
    // cv::imshow("camera", src);
  }
  camera_img = src.clone();
  return true;
}



bool CameraContainer::run(const string &file_name)
{
  // cout << __func__ << " start!!!" << endl;
  cout << "file name is " << file_name << endl;
  /*** 引数のファイルpathから画像を取得 ***/
  cv::Mat src = cv::imread(file_name, 1);
  if(src.empty()) {
    cout << "画像の読み込みに失敗しました" << endl;
    return -1;
  }
  if(debug_) cv::imshow("src", src);
  /*** src画像をhsv画像に変換する ***/
  cv::Mat hsv;
  if(changeHsv(src, hsv)){
    /*** 色情報でコンテナを検出 ***/
    /*** ラベリングし、矩形の大きさと矩形の比率でコンテナ認識精度を上げる ***/
    cv::Mat green_binary_img, blue_binary_img;
    vector<cv::Rect> green_rects, blue_rects;
    colorDetector(hsv, blue_binary_img, blue_hsv_param_);
    labeling(blue_binary_img, blue_rects, b_area_per_);
    colorDetector(hsv, green_binary_img, green_hsv_param_);
    labeling(green_binary_img, green_rects, g_area_per_);
    if(!blue_rects.empty()){
      /*** 矩形が複数あった場合 ***/
      if(green_rects.size() > 1){ //緑コンテナが複数個
      // cornerDetection(src, green_binary_img, green_rects, pt);
        selectOneRectangle(green_rects);
      }
      if(blue_rects.size() > 1){ //青コンテナが複数個
        // cornerDetection(src, blue_binary_img, blue_rects, pt);
        selectOneRectangle(blue_rects);
      }
      /*** 矩形描画 ***/
      for (auto &rect : green_rects){
        rectangle(src, rect, cv::Scalar(0, 255, 0), 2);
        /*** 矩形の端からheight_line_pixel内側の部分に赤線を描画 ***/
        cv::line(src, cv::Point(rect.x + height_line_, rect.y), cv::Point(rect.x + height_line_, rect.y + rect.height), cv::Scalar(0, 0, 255), 1, 1);
      }
      for (auto &rect : blue_rects){
        rectangle(src, rect, cv::Scalar(255, 0, 0), 2);
        /*** 矩形の端からheight_line_pixel内側の部分に赤線を描画 ***/
        cv::line(src, cv::Point(rect.x + height_line_, rect.y), cv::Point(rect.x + height_line_, rect.y + rect.height), cv::Scalar(0, 0, 255), 1, 1);
      }
    }
    /*** 画像表示 ***/
    cv::resize(src, src, cv::Size(), 0.5, 0.5);
    cv::imshow("camera", src);
  }
  cv::waitKey(0);
  return true;
}

}//namespace

string getFileName(int &num)
{
  /*** iniファイルの読み込み ***/
  prop::ptree pt;
  prop::read_ini("../cfg/lidar_container.ini", pt);

  string folder_path, file_img_path;
  if(auto v = pt.get_optional<string>("Parameter.FolderPath")) folder_path = v.get();
  /*** 画像ファイル名を取得 ***/
  std::ostringstream img_name;
  img_name << folder_path << "/../image/" << std::setw(6) << std::setfill('0') << num << ".png"; //serwは桁指定、setfileは桁の空いている部分を何で埋めるか
  file_img_path = img_name.str();
  if(file_img_path.empty()){
    cerr << "Don't get image file path" << endl;
  }
  return file_img_path;
}

/*** テスト用 ***/
int main(int argc, char** argv)
{
  container::CameraContainer camera_container;
  for(int num = 0; ;num++){
    string file_img_path = getFileName(num);
    camera_container.run(file_img_path);
  }
  return 0;
}