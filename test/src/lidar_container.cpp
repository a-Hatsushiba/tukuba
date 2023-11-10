#include "lidar_container.hpp"

using namespace std;
namespace prop = boost::property_tree;

namespace container
{

LiDARContainer::LiDARContainer()
{
  initParam();
}


LiDARContainer::~LiDARContainer()
{

}


bool LiDARContainer::initParam()
{
  /*** iniファイルの読み込み ***/
  prop::ptree pt;
  prop::read_ini("../cfg/lidar_container.ini", pt);

  if(auto v = pt.get_optional<string>("Parameter.PCDPath")) pcd_path_ = v.get();
  if(auto v = pt.get_optional<string>("Parameter.FileFormat")) file_format_ = v.get();
  if(auto v = pt.get_optional<string>("Parameter.ImagePath")) image_path_ = v.get();
  if(auto v = pt.get_optional<int>("Parameter.Range")) range_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.Scale")) scale_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.Pixel")) pixel_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.Accept")) accept_ = v.get();
  if(auto v = pt.get_optional<int>("Parameter.Angle")) angle_ = v.get();
  if(auto v = pt.get_optional<int>("Parameter.AngleOffset")) angle_offset_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.LiDARHeight")) lidar_height_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.Correction1")) correction_1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.Correction2")) correction_2_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.CorrectionZ")) correction_z_ = v.get();
  if(auto v = pt.get_optional<int>("Parameter.Debug")) debug_ = v.get();
}


bool LiDARContainer::getFileName(const int &num, string &file_path, string &file_img_path)
{
  // cout << __func__ << " start!" << endl;
  /*** 点群ファイル名を取得 ***/
  std::ostringstream name;
  name << pcd_path_ << std::setw(6) << std::setfill('0') << num << "." << file_format_; //serwは桁指定、setfileは桁の空いている部分を何で埋めるか
  file_path = name.str();
  if(file_path.empty()){
    cerr << "Don't get file path" << endl;
    return false;
  }
  if(debug_) cout << "file_path = " << file_path << endl;
  /*** 画像ファイル名を取得 ***/
  std::ostringstream img_name;
  // img_name << folder_path_ << "/../image/" << std::setw(6) << std::setfill('0') << num << ".png"; //serwは桁指定、setfileは桁の空いている部分を何で埋めるか
  img_name << image_path_ << std::setw(6) << std::setfill('0') << num << ".png"; //serwは桁指定、setfileは桁の空いている部分を何で埋めるか
  file_img_path = img_name.str();
  if(file_img_path.empty()){
    cerr << "Don't get image file path" << endl;
    return false;
  }
  if(debug_) cout << "file_img_path = " << file_img_path << endl;

  return true;
}


bool LiDARContainer::loadFilePcd(const string &file_path)
{
  // cout << __func__ << " start!" << endl;
  cout << "file_path = " << file_path << endl;
  ifstream ifs(file_path);
  if(!ifs){
    cerr << "Failed to open file" << endl;
    return false;
  }
  if(debug_) cout << "Succeeded to file open" << endl;
  /*** pcdファイルのを１行ずつ読み込む ***/
  string line, value_s;
  double value_d;
  vector<double> value_array;
  /*** 前のファイルデータは削除 ***/
  file_value_.clear();
  /*** ファイルデータ読み込み開始 ***/
  int now = 0;
  while(getline(ifs, line)){ //改行ごと最後の行まで
    now++;
    if(debug_) cout << "now = " << now << endl;
    /*** 9行目までは読み飛ばす ***/
    if(now <= 9) continue;
    /*** " (スペース)"区切りでvalue_arrayに値を入れる ***/
    istringstream iss(line);
    while(getline(iss, value_s, ' ')){
      if(debug_ && now < 10) cout << "value_d= " << now << endl;
      value_d = boost::lexical_cast<double>(value_s);
      value_array.push_back(value_d);
    }
    if(debug_ && now < 10) cout << now << "(x, y, z, e) = " << value_array[0] << ", " << value_array[1] << ", " << value_array[2] << ", " << value_array[3] << endl;
    file_value_.push_back(value_array);

    value_array.clear();
  }
}


bool LiDARContainer::loadFileCsv(const string &file_path)
{
  // cout << __func__ << " start!" << endl;
  cout << "file_path = " << file_path << endl;
  ifstream ifs(file_path);
  if(!ifs){
    cerr << "Failed to open file" << endl;
    return false;
  }
  if(debug_) cout << "Succeeded to file open" << endl;
  /*** csvファイルのを１行ずつ読み込む ***/
  string line, value_s;
  double value_d;
  vector<double> value_array;
  int now = 0;
  /*** 前のファイルデータは削除 ***/
  file_value_.clear();
  /*** ファイル読み込みスタート ***/
  while(getline(ifs, line)){ //改行ごと最後の行まで
    now++;
    if(debug_) cout << "now = " << now << endl;
    // 改行コードを削除
    if (line[line.size()-1] == '\n') line.erase(line.size()-1);
    if (line[line.size()-1] == '\r') line.erase(line.size()-1);
    /*** 2行目までは読み飛ばす ***/
    if(now <= 2) continue;
    /*** " (スペース)"区切りでvalue_arrayに値を入れる ***/
    istringstream iss(line);
    while(getline(iss, value_s, ',')){
      if(debug_ && now < 4) cout << "value_s= " << value_s << endl;
      boost::algorithm::erase_all(value_s, " ");
      value_d = boost::lexical_cast<double>(value_s);
      if(debug_ && now < 4) cout << "value_d= " << value_d << endl;
      value_array.push_back(value_d);
    }
    swap(value_array[3], value_array[4]);
    if(debug_ && now < 10) cout << now << "(x, y, z, e, r) = " << value_array[0] << ", " << value_array[1] << ", " << value_array[2] << ", " << value_array[3] << ", " << value_array[4] << endl;
    file_value_.push_back(value_array);

    value_array.clear();
  }
}


bool LiDARContainer::getPower()
{
  // cout << __func__ << " start!" << endl;
  int power = 1;
  for(double rate = pixel_; (int)rate == 0 && power < 100000; rate = rate * 10){
    power *= 10;
  }
  power_ = power;
  if(debug_) cout << "power_ = " << power_ << endl;
}


bool LiDARContainer::create2DImgXY(cv::Mat &result_img)
{
  // cout << __func__ << " start!" << endl;
  /*** 周囲(パラメータの値m)程度を見る ***/
  result_img = cv::Mat(range_*2*power_+1, range_*2*power_+1, CV_8UC1, cv::Scalar(0));
  // /*** 1m間隔の円を引く ***/
  // for(int i = range_/5; i > 0; i--){
  //   cv::circle(result_img, cv::Point(range_*power_, range_*power_), power_*1*i, cv::Scalar(200), 1, 4);
  // }
  /* 真ん中の点 */
  result_img.at<uchar>(range_*power_, range_*power_) = 200;
  cv::line(result_img, cv::Point(result_img.cols/2, result_img.rows/2), cv::Point(result_img.cols-1, result_img.rows/2), 100, 1);
  double x, y;
  double z;
  // double angle_view = M_PI / 6;
  for(int i = 0, size = file_value_.size(); i < size; i++){
    x = file_value_[i][0] * power_;
    y = file_value_[i][1] * power_;
    z = file_value_[i][2] + lidar_height_;
    // e = file_value[i][3];
    if(debug_ && i < 10) cout << "(x, y) = " << x << ", " << y << endl;
    if((x >= -range_*power_ && x <= range_*power_) && (y >= -range_*power_ && y <= range_*power_) && (z > 0.2)){
      // /** 自分の正面から左右angle_viewずつの部分だけ切りとる **/
      if(atan2(y, x) >= (- (angle_/2 - angle_offset_) * M_PI / 180) && atan2(y, x) <= ((angle_/2 + angle_offset_) * M_PI / 180)){
        // result_img.at<uchar>(range_*2*power_ - (range_*power_ + y), range_*power_ + x) = 255;
        result_img.at<uchar>(range_*2*power_ - (range_*power_ + x), range_*2*power_ - (range_*power_ + y)) = 255;
      }
    }
  }
}


bool LiDARContainer::create2DImgXZ(cv::Mat &result_img)
{
  // cout << __func__ << " start!" << endl;
  int row = 2;
  result_img = cv::Mat(row*2*power_+1, range_*power_+1, CV_8UC1, cv::Scalar(0));
  cv::line(result_img, cv::Point(0, result_img.rows/2), cv::Point(result_img.cols-1, result_img.rows/2), 150, 1);
  double angle_view = M_PI / 6;
  int x, y;
  double z, r;
  for(int i = 0, size = file_value_.size(); i < size; i++){
    x = file_value_[i][0] * power_;
    y = file_value_[i][1] * power_;
    z = file_value_[i][2] * power_ - lidar_height_ * power_;
    r = sqrt((x * x) + (y * y)) / power_;
    if((atan2(x, y) >= (M_PI/2 - angle_view) && atan2(x, y) <= (M_PI/2 + angle_view)) && (r <= range_) && ( -row*power_ <= z && z <= row*power_ )){
      /** 自分の正面から左右45℃ずつの部分だけ切りとる **/
      result_img.at<uchar>(row*2*power_ - (row*power_ + z), x) = 255;
    }
  }
  return true;
}


// bool LiDARContainer::createDepthImg(cv::Mat &depth_img)
// {
// 
//   int row = 5;
//   double angle_view = M_PI / 6;
//   double angle_view_wide = range_ * sin(angle_view) * power_;
//   // double angle_view_wide = M_PI * range_ * angle_view / 2 / (2 * M_PI) * power_;
//   depth_img = cv::Mat(row*2*power_+1, angle_view_wide*2+1, CV_64FC1, cv::Scalar(0));
//   // cv::line(depth_img, cv::Point(0, depth_img.rows/2), cv::Point(depth_img.cols-1, depth_img.rows/2), 150, 1);
//   double x, y, z, r_xy, r_xyz;
//   for(int i = 0, size = file_value_.size(); i < size; i++){
//     x = file_value_[i][0] * power_;
//     y = file_value_[i][1] * power_;
//     z = file_value_[i][2] * power_ - lidar_height_ * power_;
//     r_xy = sqrt((x * x) + (y * y)) / power_;
//     r_xyz = sqrt((x * x) + (y * y) + (z * z)) / power_;
//     if(atan2(x, y) >= (M_PI_2 - angle_view) && atan2(x, y) <= (M_PI_2 + angle_view)){ //見る範囲の制限
//       if(depth_img.at<double>(depth_img.rows -  (depth_img.rows / 2 + z*(range_/r_xyz)), depth_img.cols - (angle_view_wide + y*(range_/r_xy))) == 0
//         || depth_img.at<double>(depth_img.rows - (depth_img.rows / 2 + z*(range_/r_xyz)), depth_img.cols - (angle_view_wide + y*(range_/r_xy))) > r_xy){ //距離の話
//         depth_img.at<double>(depth_img.rows - (depth_img.rows / 2 + z*(range_/r_xyz)), depth_img.cols - (angle_view_wide + y*(range_/r_xy))) = r_xyz;
//       }
//     }
//   }
//   return true;
// }


// bool LiDARContainer::createDepthImg(cv::Mat &depth_img)
// {
//   int row = 5;
//   double angle_view = (angle_ / 2) * (M_PI / 180);
//   double angle_view_wide = range_ * sin(angle_view) * power_;
//   depth_img = cv::Mat(row*2*power_+1, angle_view_wide*2+1, CV_64FC1, cv::Scalar(0));
//   double x, y, z, r_xy, r_xyz, theta, correction;
//   cv::line(depth_img, cv::Point(depth_img.cols/2, 0), cv::Point(depth_img.cols/2, depth_img.rows-1), cv::Scalar(255, 255, 255), 1);
//   for(int i = 0, size = file_value_.size(); i < size; i++){
//     x = file_value_[i][0] * power_;
//     y = file_value_[i][1] * power_;
//     z = file_value_[i][2] * power_ - lidar_height_ * power_;
//     r_xy = sqrt((x * x) + (y * y)) / power_;
//     r_xyz = sqrt((x * x) + (y * y) + (z * z)) / power_;
//     theta = atan2(x, y);
//     correction = 0.3 * power_ * (r_xy/range_);
//     y += correction;
//     if(theta >= (M_PI_2 - angle_view) && theta <= (M_PI_2 + angle_view) && (r_xy <= range_)){ //見る範囲の制限
//       if(depth_img.at<double>(depth_img.rows -  (depth_img.rows / 2 + z*(range_/r_xy)), depth_img.cols - (angle_view_wide + y*(range_/r_xy))) == 0
//         || depth_img.at<double>(depth_img.rows - (depth_img.rows / 2 + z*(range_/r_xy)), depth_img.cols - (angle_view_wide + y*(range_/r_xy))) > r_xyz){ //距離の話
//         if(1) depth_img.at<double>(depth_img.rows - (depth_img.rows / 2 + z*(range_/r_xy)), depth_img.cols - (angle_view_wide + y*(range_/r_xy))) = r_xyz;
//       }
//     }
//   }
//   return true;
// }


// y座標はだいたい合うようになった->次はz座標
bool LiDARContainer::createDepthImg(cv::Mat &depth_img)
{
  // cout << __func__ << " start!" << endl;
  double view_angle = (angle_ / 2)*(M_PI / 180); //見る範囲
  double col_size = range_ * sin(view_angle);
  // double col_size = range_ / 2 * sin(view_angle);
  int row_size = 5;
  /*** 画像の大きさ等を定義 ***//*** 1画素にx,yの情報が入っている ***/
  depth_img = cv::Mat(row_size * 2 * power_ + 1, col_size * 2 * power_ + 1, CV_64FC3, cv::Scalar(0));
  /*** file_value_から値を取り出す ***/
  double x, y, z, atan2_xy, r_xy, y_cor;
  for(int i=0, size = file_value_.size(); i < size; i++){
    x = file_value_[i][0];
    y = file_value_[i][1];
    z = file_value_[i][2];

    r_xy = sqrt(x*x + y*y);
    y_cor = y;
    /*** カメラ画像に合わせるために調節 ***/
    y_cor += correction_1_ * (r_xy/range_);//y座標合わせるための補正値(全体的に左へ)
    // y_cor -= correction_2_ * (y/col_size);//y座標が0から離れているものを0側へ近づける
    y_cor -= correction_2_ * (y_cor/(col_size + correction_1_ * (r_xy/range_)));//y座標が0から離れているものを0側へ近づける
    z -= lidar_height_;
    atan2_xy = atan2(x, y_cor);
    double access_pixel_val = depth_img.at<cv::Vec3d>((depth_img.rows - 1) - ((row_size + z*(correction_z_ * range_/r_xy)) * power_),
                                                   (depth_img.cols - 1) - ((col_size + y_cor*(range_/r_xy)) * power_))[2];
    if(((M_PI_2 - view_angle) <= atan2_xy) && (atan2_xy <= (M_PI_2 + view_angle))){//画角の制限
      if(r_xy <= range_ && (-lidar_height_ + 0.1 <= z && z <= row_size)){//range_以内の点群を表示&zは0以上
        if(access_pixel_val == 0 || access_pixel_val > r_xy){ //画素に何も入っていないor画素の中身による
          depth_img.at<cv::Vec3d>((depth_img.rows - 1) - ((row_size + z*(correction_z_ * range_/r_xy))) * power_,
                                  (depth_img.cols - 1) - ((col_size + y_cor*(range_/r_xy)) * power_)) = cv::Vec3d(x, y, r_xy);
        }
      }
    }
  }

  return true;
}

/*** xの情報の線形補間を行う ***/
cv::Vec3d LiDARContainer::linerInterporation(cv::Mat &src, const cv::Point &pix)
{
  int pix_range = 5;
  int src_y1 = 0, src_y2 = 0;
  double x1 = 0, x2 = 0;
  double y1 = 0, y2 = 0;
  double r1 = 0, r2 = 0;
  /*** yのマイナス方向 ***/
  for(int y = 1; y <= pix_range && pix.y - y >= 0; y++){
    if(src.at<cv::Vec3d>(pix.y - y, pix.x)[2] > 0.00){
      src_y1 = y;
      x1 = src.at<cv::Vec3d>(pix.y - y, pix.x)[0];
      y1 = src.at<cv::Vec3d>(pix.y - y, pix.x)[1];
      r1 = src.at<cv::Vec3d>(pix.y - y, pix.x)[2];
    }
  }
  /*** yのプラス方向 ***/
  for(int y = 1; y <= pix_range && pix.y + y < src.rows ; y++){
    if(src.at<cv::Vec3d>(pix.y + y, pix.x)[2] > 0.00){
      src_y2 = y;
      x2 = src.at<cv::Vec3d>(pix.y + y, pix.x)[0];
      y2 = src.at<cv::Vec3d>(pix.y + y, pix.x)[1];
      r2 = src.at<cv::Vec3d>(pix.y + y, pix.x)[2];
    }
  }
  /*** 線形補間 ***/
  double x_result, y_result, r_result;
  if(src_y1 != 0 && src_y2 != 0 && abs(x1 - x2) <= accept_){
    x_result = x1 * ((double)src_y2/(src_y1+src_y2)) + x2 *((double)src_y1/(src_y1+src_y2));
    y_result = y1 * ((double)src_y2/(src_y1+src_y2)) + y2 *((double)src_y1/(src_y1+src_y2));
    r_result = r1 * ((double)src_y2/(src_y1+src_y2)) + r2 *((double)src_y1/(src_y1+src_y2));
    // cout << "src_y2/(src_y1+src_y2) : " << src_y2/(src_y1+src_y2) << ", src_y1/(src_y1+src_y2) : " << src_y1/(src_y1+src_y2) << endl;
    // cout << "r1: " << r1 << ", r2: " << r2 << endl;
    // cout << "r_result : " << r_result << endl;
  }
  else{
    x_result = 0;
    y_result = 0;
    r_result = 0;
  }
  return cv::Vec3d(x_result, y_result, r_result);
}

bool LiDARContainer::createDepthColorImg(cv::Mat &depth_img, cv::Mat &depth_label_img)
{
  // cout << __func__ << " start!" << endl;
  /** double型のMatに色つける **/
  depth_label_img = cv::Mat(depth_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  /*** 近いほどHが0に近く、遠いほどHが120に近くなるようにする。 depth_ptr[x]にそのままの距離の値が入っているのか違うのかはわからない***/
  for(int y = 0, rows = depth_label_img.rows; y < rows; y++){
    auto *depth_ptr = depth_img.ptr<cv::Vec3d>(y);
    auto *depth_label_ptr = depth_label_img.ptr<cv::Vec3b>(y);
    for(int x = 0, cols = depth_label_img.cols; x < cols; x++){
      /** ここで線形補間を行う **/
      if(depth_ptr[x][0] == 0.00){
        depth_ptr[x] = linerInterporation(depth_img, cv::Point(x, y));
        // if(depth_ptr[x][2] != 0) cout << "depth_ptr[x][2] : " << depth_ptr[x][2] << endl;
      }
      /** そのままの距離の値が入っていると仮定する **///yamlにパラメータとして入れてもいいかも
      if(depth_ptr[x][2] > 0 && depth_ptr[x][2] <= range_) depth_label_ptr[x] = cv::Vec3b((120.0 / range_) * depth_ptr[x][2], 255, 255);
    }
  }
  cv::cvtColor(depth_label_img, depth_label_img, cv::COLOR_HSV2BGR);
  return true;
}

bool LiDARContainer::createRefrectImg(cv::Mat &ref_img)
{
  // cout << __func__ << " start!" << endl;
  double view_angle = (angle_ / 2)*(M_PI / 180); //見る範囲
  double col_size = range_ * sin(view_angle);
  // double col_size = range_ / 2 * sin(view_angle);
  int row_size = 5;
  /*** 画像の大きさ等を定義 ***//*** 1画素にx,yの情報が入っている ***/
  ref_img = cv::Mat(row_size * 2 * power_ + 1, col_size * 2 * power_ + 1, CV_64FC3, cv::Scalar(0));
  /*** file_value_から値を取り出す ***/
  double x, y, z, e, atan2_xy, r_xy, y_cor;
  for(int i=0, size = file_value_.size(); i < size; i++){
    x = file_value_[i][0];
    y = file_value_[i][1];
    z = file_value_[i][2];
    e = file_value_[i][3];

    r_xy = sqrt(x*x + y*y);
    y_cor = y;
    /*** カメラ画像に合わせるために調節 ***/
    y_cor += correction_1_ * (r_xy/range_);//y座標合わせるための補正値(全体的に左へ)
    // y_cor -= correction_2_ * (y/col_size);//y座標が0から離れているものを0側へ近づける
    y_cor -= correction_2_ * (y_cor/(col_size + correction_1_ * (r_xy/range_)));//y座標が0から離れているものを0側へ近づける
    z -= lidar_height_;
    atan2_xy = atan2(x, y_cor);
    double access_pixel_val = ref_img.at<cv::Vec3d>((ref_img.rows - 1) - ((row_size + z*(correction_z_ * range_/r_xy)) * power_),
                                                   (ref_img.cols - 1) - ((col_size + y_cor*(range_/r_xy)) * power_))[2];
    if(((M_PI_2 - view_angle) <= atan2_xy) && (atan2_xy <= (M_PI_2 + view_angle))){//画角の制限
      if(r_xy <= range_ && (-lidar_height_ + 0.1 <= z && z <= row_size)){//range_以内の点群を表示&zは0以上
        if(access_pixel_val == 0 || access_pixel_val > e){ //画素に何も入っていないor画素の中身による
          ref_img.at<cv::Vec3d>((ref_img.rows - 1) - ((row_size + z*(correction_z_ * range_/r_xy))) * power_,
                                  (ref_img.cols - 1) - ((col_size + y_cor*(range_/r_xy)) * power_)) = cv::Vec3d(x, y, e);
        }
      }
    }
  }
  return true;
}

bool LiDARContainer::createRefrectColorImg(cv::Mat &ref_img, cv::Mat &ref_label_img)
{
  // cout << __func__ << " start!" << endl;
  /** double型のMatに色つける **/
  ref_label_img = cv::Mat(ref_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  /*** 近いほどHが0に近く、遠いほどHが120に近くなるようにする。 depth_ptr[x]にそのままの距離の値が入っているのか違うのかはわからない***/
  for(int y = 0, rows = ref_label_img.rows; y < rows; y++){
    auto *depth_ptr = ref_img.ptr<cv::Vec3d>(y);
    auto *depth_label_ptr = ref_label_img.ptr<cv::Vec3b>(y);
    for(int x = 0, cols = ref_label_img.cols; x < cols; x++){
      // /** ここで線形補間を行う **/
      // if(depth_ptr[x][0] == 0.00){
      //   depth_ptr[x] = linerInterporation(ref_img, cv::Point(x, y));
      //   // if(depth_ptr[x][2] != 0) cout << "depth_ptr[x][2] : " << depth_ptr[x][2] << endl;
      // }
      /** そのままの距離の値が入っていると仮定する **///yamlにパラメータとして入れてもいいかも
      if(depth_ptr[x][2] > 0 && depth_ptr[x][2] <= range_) depth_label_ptr[x] = cv::Vec3b(120 - (120.0 / 1) * depth_ptr[x][2], 255, 255);
    }
  }
  cv::cvtColor(ref_label_img, ref_label_img, cv::COLOR_HSV2BGR);
  return true;
}

/*** 特定距離を同一とみなしてラベリングする ***/
bool LiDARContainer::labelingDepth(cv::Mat &src, vector<cv::Rect> &rects)
{
  // cout << __func__ << " start!" << endl;
  struct memory{
    int flag;
    int x_start;
    int x_end;
    int y_start;
    int y_end;
  };

  cv::Mat label = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
  std::vector<int> search_store; //すでに探索済みの8近傍を記録
  std::vector<std::pair<int, int>> remember1; //置き換えを記録する用
  std::vector<std::pair<int, int>> remember2; //2回目の書き換え用
  int count = 0;
  int min;

  /*** 注目画素と距離が離れていない(0.5m以内の)部分にラベル付を行う ***/
  for(int src_y = 0;src_y < src.rows; src_y++){
    /*** 行の先頭のポインタを取得 ***/
    double *ptr_src = src.ptr<double>(src_y);
    double *ptr_src_m = src.ptr<double>(src_y-1);
    uchar *ptr_label = label.ptr<uchar>(src_y);
    uchar *ptr_label_m = label.ptr<uchar>(src_y-1);
    for(int src_x = 0;src_x < src.cols; src_x++){
      if(ptr_src[src_x] != 0.0){ //探索した画素値が0.0でない
        if(src_x == 0 && src_y == 0){ //(0,0)
          ptr_label[src_x] = ++count;
          remember1.push_back(std::make_pair(count, count));
        }else if(src_y == 0){ //(x,0)
          if(ptr_src[src_x-1] != 0 && distanceComp(ptr_src[src_x], ptr_src[src_x-1])){
            ptr_label[src_x] = ptr_label[src_x - 1];
          }else{
            ptr_label[src_x] = ++count;
            remember1.push_back(std::make_pair(count, count));
          }
        }else if(src_x == 0){ //(0,y)
          for(int search_i = 0; search_i < 2; search_i++){ //8近傍に値があるかどうかを探索し、あればsearch_storeに格納
            if(ptr_src_m[src_x + search_i] != 0 && distanceComp(ptr_src[src_x], ptr_src_m[src_x + search_i])){
              search_store.push_back(ptr_label_m[src_x  + search_i]);
            }
          }
          if(!search_store.empty()){
            min = *min_element(search_store.begin(), search_store.end()); //8近傍にある値の最小値を取得
            ptr_label[src_x] = min;
            for(int i=0; i < search_store.size(); i++){ //8近傍にある大きなラベルを統合する時用に記録
              if(search_store[i] > min && remember1[search_store[i]-1].second > min){
                remember1[search_store[i]-1].second = min;
              }
            }
            search_store.clear();
          }else{
            ptr_label[src_x] = ++count;
            remember1.push_back(std::make_pair(count, count));
          }
        }else if(src_x == src.cols-1){ //(src.cols-1, y)
          for(int search_i = 0; search_i < 2; search_i++){ //8近傍に値があるかどうかを探索し、あればsearch_storeに格納
            if(ptr_src_m[src_x-1 + search_i] != 0 && distanceComp(ptr_src[src_x], ptr_src_m[src_x-1 + search_i])){
              search_store.push_back(ptr_label_m[src_x - 1 + search_i]);
            }
          }
          if(ptr_src[src_x-1] != 0 && distanceComp(ptr_src[src_x], ptr_src[src_x-1])){
            search_store.push_back(ptr_label[src_x - 1]);
          }
          if(!search_store.empty()){
            min = *min_element(search_store.begin(), search_store.end()); //8近傍にある値の最小値を取得
            ptr_label[src_x] = min;
            for(int i=0; i < search_store.size(); i++){ //8近傍にある大きなラベルを統合する時用に記録
              if(search_store[i] > min && remember1[search_store[i]-1].second > min){
                remember1[search_store[i]-1].second = min;
              }
            }
            search_store.clear();
          }else{
            ptr_label[src_x] = ++count;
            remember1.push_back(std::make_pair(count, count));
          }
        }else{ //上記以外の場合
          for(int search_i = 0; search_i < 3; search_i++){ //8近傍に値があるかどうかを探索し、あればsearch_storeに格納
            if(ptr_src_m[src_x-1 + search_i] != 0 && distanceComp(ptr_src[src_x], ptr_src_m[src_x-1 + search_i])){
              search_store.push_back(ptr_label_m[src_x-1 + search_i]);
            }
          }
          if(ptr_src[src_x-1] != 0 && distanceComp(ptr_src[src_x], ptr_src[src_x-1])){
            search_store.push_back(ptr_label[src_x - 1]);
          }
          if(!search_store.empty()){
            min = *min_element(search_store.begin(), search_store.end()); //8近傍にある値の最小値を取得
            ptr_label[src_x] = min;
            for(int i=0; i < search_store.size(); i++){ //8近傍にある大きなラベルを統合する時用に記録
              if(search_store[i] > min && remember1[search_store[i]-1].second > min){
                remember1[search_store[i]-1].second = min;
              }
            }
            search_store.clear();
          }else{
            ptr_label[src_x] = ++count;
            remember1.push_back(std::make_pair(count, count));
          }
        }
      }
    }
  }
  /*** remember1の中身を整理する ***/
  for(int i = 0; i < remember1.size(); i++){
    if(remember1[i].first != remember1[i].second){
      int rem = remember1[i].second;
      while(remember1[rem - 1].first != remember1[rem - 1].second){
         rem = remember1[rem - 1].second;
      }
      remember1[i].second = remember1[rem - 1].second;
    }
  }
  /*** remember1の中身確認用 ***/
  // std::cout << remember1.size() << std::endl;
  // for(int i = 0; i < remember1.size(); i++){
  //   std::cout << remember1[i].first << ", " << remember1[i].second << std::endl;
  // }
  /*** 1回目のラベルの振り直し ***/
  for (int label_y = 0; label_y < label.rows; label_y++){
    uchar *ptr_label = label.ptr(label_y);
    for (int label_x = 0; label_x < label.cols; label_x++){
      if(ptr_label[label_x] != 0 && remember1[ptr_label[label_x]-1].first != remember1[ptr_label[label_x]-1].second){
        ptr_label[label_x] = remember1[ptr_label[label_x]-1].second;
      }
    }
  }
  /*** 2回目の振り直し用にremember1からremember2に変更 ***/
  count = 0;
  remember2.push_back(std::make_pair(1, ++count));
  for(int i=0; i < remember1.size(); i++){
    if(remember1[i].second > remember2[count-1].first){
      remember2.push_back(std::make_pair(remember1[i].second, ++count));
    }
  }
  /*** remember2確認用 ***/
  // for(int i = 0; i < remember2.size(); i++){
  //   std::cout << remember2[i].first << ", " << remember2[i].second << std::endl;
  // }
 
  /*** 2回目のラベルの振り直し ***/
  for (int label_y = 0; label_y < label.rows; label_y++){
    uchar *ptr_label = label.ptr(label_y);
    for (int label_x = 0; label_x < label.cols; label_x++){
      if(ptr_label[label_x] != 0){
        for(int i=0; i < remember1.size(); i++){
          if(ptr_label[label_x] == remember2[i].first){
            ptr_label[label_x] = remember2[i].second;
          }
        }
      }
    }
  }
 
  /*** それぞれの領域の情報を取る ***//* x, y, width, height */
  memory memo[remember2.size()] = {}; //今回は必要分のものしか入れないので大きさを宣言してしまう
  for (int label_y = 0; label_y < label.rows; label_y++){
    uchar *ptr_label = label.ptr(label_y);
    for (int label_x = 0; label_x < label.cols; label_x++){
      if(ptr_label[label_x] != 0){
        if(memo[ptr_label[label_x]-1].flag == 0){
          memo[ptr_label[label_x]-1].x_start = label_x;
          memo[ptr_label[label_x]-1].x_end = label_x;
          memo[ptr_label[label_x]-1].y_start = label_y;
          memo[ptr_label[label_x]-1].y_end = label_y;
          memo[ptr_label[label_x]-1].flag = 1;
        }else{
          if(memo[ptr_label[label_x]-1].x_start > label_x){
            memo[ptr_label[label_x]-1].x_start = label_x;
          }
          if(memo[ptr_label[label_x]-1].x_end < label_x){
            memo[ptr_label[label_x]-1].x_end = label_x;
          }
          if(memo[ptr_label[label_x]-1].y_start > label_y){
            memo[ptr_label[label_x]-1].y_start = label_y;
          }
          if(memo[ptr_label[label_x]-1].y_end < label_y){
            memo[ptr_label[label_x]-1].y_end = label_y;
          }
        }
      }
    }
  }
 
  /*** rectに(x, y, width, height)の値をいれる ***/
  rects.resize(remember2.size());
  for(int i=0, size = remember2.size(); i < size; i++){
    // cout << "remember2.size : " << size << endl;
    rects[i] = cv::Rect(memo[i].x_start, memo[i].y_start, memo[i].x_end - memo[i].x_start, memo[i].y_end - memo[i].y_start);
  }

  return true;
}


bool LiDARContainer::distanceComp(const double &target, const double &comp_pix)
{
  // cout << abs(target - comp_pix) << endl;
  if(abs(target - comp_pix) > accept_) return false;
  return true;
}


bool LiDARContainer::run()
{
  // cout << __func__ << " start!" << endl;
  string file_path, file_img_path;
  bool stop_flag = true, load_flag;
  for(int num = 0;  ;num++){
    /*** ファイル名読み込み ***/
    getFileName(num, file_path, file_img_path);
    /*** ファイル情報読み込み ***/
    if(file_format_ == "pcd") load_flag = loadFilePcd(file_path);
    else if(file_format_ == "csv") load_flag = loadFileCsv(file_path);
    else return false;
    if(load_flag){
      /*** 1ピクセルあたり ***/
      getPower();
      /*** 点群情報から2D画像を作成 ***/
      cv::Mat xy_img, xz_img;
      cv::Mat depth_img, depth_color_img;
      create2DImgXY(xy_img);
      create2DImgXZ(xz_img);
      createDepthImg(depth_img);
      createDepthColorImg(depth_img, depth_color_img);
      /*** 距離画像でラベリング ***/
      vector<cv::Rect> depth_rects;
      if(labelingDepth(depth_img, depth_rects)){
        cout << "size : " << depth_rects.size() << endl;
        /*** ラベリング領域の描画 ***/
        for (auto &rect : depth_rects)
          rectangle(depth_color_img, rect, cv::Scalar(0, 0, 255), 1);
      }
      depth_rects.clear();
      /*** 画像表示 ***/
      cv::resize(xy_img, xy_img, cv::Size(), scale_, scale_, cv::INTER_NEAREST);
      cv::imshow("2d(x, y)", xy_img);
      cv::imshow("2d(x, z)", xz_img);
      cv::resize(depth_img, depth_img, cv::Size(), 3, 3, cv::INTER_NEAREST);
      cv::imshow("2d(depth)", depth_img);
      cv::resize(depth_color_img, depth_color_img, cv::Size(), 3, 3, cv::INTER_NEAREST);
      cv::imshow("2d(color)", depth_color_img);

      /*** 画像表示時のキーの動き ***/
      int key;
      if(stop_flag) key = cv::waitKey(0);
      if(!stop_flag) key = cv::waitKey(50);
      if(key == 'q') break;
      else if(key == 'p') std::cout << file_path << std::endl;
      else if(key == 'b') num = num - 2;
      else if(key == 's'){
        if(stop_flag == 0) stop_flag = 1;
        else stop_flag = 0;
      }
    }
    else break;
  }
  return true;
}


bool LiDARContainer::run(string &file_path, cv::Mat &lidar_img, cv::Mat &lidar_info)
{
  // string file_path, file_img_path;
  bool load_flag;
  /*** ファイル情報読み込み ***/
  if(file_format_ == "pcd") load_flag = loadFilePcd(file_path);
  else if(file_format_ == "csv") load_flag = loadFileCsv(file_path);
  else return false;
  if(load_flag){
    /*** 1ピクセルあたり ***/
    getPower(); //これ入れずに普通に割ればいいのでは？？？
    /*** 点群情報から2D画像を作成 ***/
    cv::Mat xy_img, xz_img;
    cv::Mat depth_img, depth_color_img, ref_img, ref_color_img;
    create2DImgXY(xy_img);
    create2DImgXZ(xz_img);
    createDepthImg(depth_img);
    createDepthColorImg(depth_img, depth_color_img);
    // createRefrectImg(ref_img);
    // createRefrectColorImg(ref_img, ref_color_img);
    /*** 距離画像でラベリング ***/
    // vector<cv::Rect> depth_rects;
    // if(labelingDepth(depth_img, depth_rects)){
    //   /*** ラベリング領域の描画 ***/
    //   for (auto &rect : depth_rects)
    //     rectangle(depth_color_img, rect, cv::Scalar(0, 0, 255), 1);
    // }
    // depth_rects.clear();
    /*** 画像表示 ***/
    /* xy */
    // cv::resize(xy_img, xy_img, cv::Size(), scale_, scale_, cv::INTER_NEAREST);
    // cv::imshow("2d(x, y)", xy_img);
    /* xz */
    cv::imshow("2d(x, z)", xz_img);
    /* yz(depth) */
    // cv::resize(depth_img, depth_img, cv::Size(), 3, 3, cv::INTER_NEAREST);
    cv::imshow("2d(depth)", depth_img);
    /* depth->color */
    // cv::resize(depth_color_img, depth_color_img, cv::Size(), 3, 3, cv::INTER_NEAREST);
    cv::imshow("2d(color)", depth_color_img);
    /* depth->color */
    // cv::resize(depth_color_img, depth_color_img, cv::Size(), 3, 3, cv::INTER_NEAREST);
    // cv::imshow("2d(ref)", ref_color_img);

    lidar_img = depth_color_img.clone();
    lidar_info = depth_img.clone();
  }
  return true;
}

}//namespace


/*** テスト用 ***/
int main(int argc, char** argv)
{
  container::LiDARContainer lidar_container;
  lidar_container.run();

  return 0;
}