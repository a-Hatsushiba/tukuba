/**
 * １．緑コンテナ検出 ー> 矩形でどこにあるか分かる
 * ２．矩形内で白色(?)の部分を検出し、マッチング開始
 **/
/* C++ */
#include<opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <algorithm>
/* boost(iniファイル読み込み) */
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

using namespace std;
using namespace cv;
namespace prop = boost::property_tree;

/*** テストのためにグローバル変数を使う ***/
vector<int> g_modo(4, 0); //a, b, c, n用
int g_count = 0;
string label_st_ = "wait";

/*** テンプレート画像の読み込み ***/
  /*** かなり良い(実行時間300ミリ秒前後) ***/
Mat temp_a =  imread("/mnt/hgfs/labo/address_label/delivery_address_a_test2.jpg", 0);
Mat temp_b =  imread("/mnt/hgfs/labo/address_label/delivery_address_b_test2.jpg", 0);
Mat temp_c =  imread("/mnt/hgfs/labo/address_label/delivery_address_c_test2.jpg", 0);


void colorDitector(const cv::Mat &src, cv::Mat &hsv, cv::Mat &result_green, prop::ptree &pt)
{
  /*** iniファイルからパラメータを読み込み ***/
  int green_hue_min_, green_hue_max_;
  int green_sat_min_, green_sat_max_;
  int green_val_min_, green_val_max_;
  int debug;
  if(auto v = pt.get_optional<int>("GreenParameter.HueMin")) green_hue_min_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.HueMax")) green_hue_max_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.SatMin")) green_sat_min_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.SatMax")) green_sat_max_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.ValMin")) green_val_min_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.ValMax")) green_val_max_ = v.get();
  if(auto v = pt.get_optional<int>("Parameter.ColorDetectorDebug")) debug = v.get();

  /*** 画像をBGR->HSVに変換 ***/
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  /*** 出力画像を0で初期化しておく ***/
  result_green = cv::Mat(src.size(), CV_8U, cv::Scalar(0));
  /*** 明度(V)のヒストグラム平坦化 ***/
  vector<cv::Mat> planes(3);
  vector<cv::Mat> hsv_merge(3);
  cv::Mat merge;
  //3つのチャンネル(H, S, V)に分離
  cv::split(hsv, planes);
  //Vのヒストグラム平坦化を行う
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
  clahe->apply(planes[2], planes[2]);
  //分離したHSVのチャンネルを合成
  hsv_merge[0] = planes[0];
  hsv_merge[1] = planes[1];
  hsv_merge[2] = planes[2];
  cv::merge(hsv_merge, hsv);
  cv::cvtColor(hsv, merge, cv::COLOR_HSV2BGR);
  if(debug) cv::imshow("hist", merge);

  /*** 下半分の範囲でしきい値内である画素を抽出 ***/ //実験のために1/3にしているが、本当は1/2
  for(int rows = src.rows, y = rows * 1 / 2 ; y < rows; y++){
    /*** 行の先頭のポインタを取得 ***/
    cv::Vec3b *ptr_hsv = hsv.ptr<cv::Vec3b>(y);
    uchar *ptr_res_green = result_green.ptr<uchar>(y);
    for(int x = 0, cols = src.cols; x < cols; x++){
      /*** 緑コンを検出 ***/
      if((green_hue_min_ <= ptr_hsv[x][0] && ptr_hsv[x][0] <= green_hue_max_)
          && (green_sat_min_ <= ptr_hsv[x][1])
          && (green_val_min_ <= ptr_hsv[x][2])){
        ptr_res_green[x] = 255;
      }
    }
  }
  if(debug){
    cv::namedWindow("green_extract", cv::WINDOW_NORMAL);
    cv::imshow("green_extract", result_green);
  }
}

void labeling(const cv::Mat &result_green, vector<cv::Rect> &green_rect, prop::ptree &pt)
{
  cv::Mat labels, stats, centroids;
  int rows = result_green.rows;
  int rows_start = rows * 1 / 2; //実験のために1/3にしているが、本当は1/2
  int rows_range = rows - rows_start;
  int write_to_file;
  int h_size_min_, w_size_min_, h_size_max_, w_size_max_;
  double rate_min_, rate_max_, g_area_per_, b_area_per_;
  /*** 新用 ***/
  int h_size_min1_, w_size_min1_, h_size_max1_, w_size_max1_;
  int h_size_min2_, w_size_min2_, h_size_max2_, w_size_max2_;
  if(auto v = pt.get_optional<double>("Parameter.HSizeMin1")) h_size_min1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMin1")) w_size_min1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.HSizeMax1")) h_size_max1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMax1")) w_size_max1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.HSizeMin2")) h_size_min2_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMin2")) w_size_min2_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.HSizeMax2")) h_size_max2_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMax2")) w_size_max2_ = v.get();

  // if(auto v = pt.get_optional<double>("Parameter.HSizeMin")) h_size_min_ = v.get();
  // if(auto v = pt.get_optional<double>("Parameter.WSizeMin")) w_size_min_ = v.get();
  // if(auto v = pt.get_optional<double>("Parameter.HSizeMax")) h_size_max_ = v.get();
  // if(auto v = pt.get_optional<double>("Parameter.WSizeMax")) w_size_max_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.RateMin")) rate_min_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.RateMax")) rate_max_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.GAreaPer")) g_area_per_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.BAreaPer")) b_area_per_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WriteToFile")) write_to_file = v.get();
  // cout << h_size_min_ << ", " << w_size_min_ << endl;
  /*** 緑コン4近傍でラベリング ***/
  int n = cv::connectedComponentsWithStats(result_green, labels, stats, centroids, 4);
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
      if(((h > h_size_min1_ && h < h_size_max1_) && (w > w_size_min1_ && w < w_size_max1_)) && (rate_min_ < ((double)w / h) && ((double)w / h) < rate_max_) && (area >= h * w *g_area_per_)){
        green_rect.push_back(cv::Rect(x, y, w, h));
        // cout << "green height = " << h << endl;
        // cout << "green width = " << w << endl;
        // cout << "green rate = " << (double)w/h << endl;
        if(write_to_file){
          ofstream ofs("./container_size.txt", ios::app);
          ofs << h  << " " << w << endl;
        }
      }
    }else if(y >= rows_start + rows_range / 2 && y < rows){
        // cout << "green[2]" << endl;
        // cout << "green height = " << h << endl;
        // cout << "green width = " << w << endl;
      if(((h > h_size_min2_ && h < h_size_max2_) && (w > w_size_min2_ && w < w_size_max2_)) && (area >= h * w *g_area_per_)){
        green_rect.push_back(cv::Rect(x, y, w, h));
        // cout << "green height = " << h << endl;
        // cout << "green width = " << w << endl;
        if(write_to_file){
          ofstream ofs("./container_size.txt", ios::app);
          ofs << h  << " " << w << endl;
        }
      }
    }
  }
}

/*** 矩形が複数個検出されてしまったときの処理 ***/
void selectOneRectangle(vector<cv::Rect> &rects)
{
  rects.erase(rects.begin(), rects.end() - 1);
}

/*** ラベルの部分だけを検出する ***/
void labelDetector(const cv::Mat &hsv, cv::Mat &label_img, const vector<cv::Rect> &rect, const cv::Mat &green_bin, prop::ptree &pt)
{
  int debug;
  if(auto v = pt.get_optional<int>("Parameter.LabelDetectorDebug")) debug = v.get();
  /*** hsvの画像をグレースケール画像にしている ***/
  cv::Mat gray, bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  /*** 緑コンテナ部分の画像だけを抜き出す ***/
  cv::Mat container = green_bin(rect[0]).clone();
  /*** 緑コンテナ矩形の左上の領域を出す ***/
  // cv::Mat container = green_bin(cv::Rect(rect[0].x, rect[0].y, rect[0].width/2, rect[0].height * 3 / 5)).clone();
  if(debug) cv::imshow("green container", container);

  /*** 縮小膨張処理を行う ***/
  dilate(container, container, cv::noArray(), cv::Point(-1, -1), 5);
  erode(container, container, cv::noArray(), cv::Point(-1, -1), 5);
  /*** 2値化 ***/
  // int binary_threshold;
  // if(auto v = pt.get_optional<int>("CharacterParameter.BinaryThreshold")) binary_threshold = v.get();
  // cv::Mat binary;
  // cv::threshold(container, binary, binary_threshold, 255, THRESH_BINARY);
  // if(debug) cv::imshow("binary", binary);

  /*** 緑コン矩形内で緑ではない部分を抜き出す(白黒反転させる) ***/
  cv::Mat binary;
  cv::bitwise_not(container, binary);
  if(debug) cv::imshow("binary", binary);


  /*** ラベリング ***/
  cv::Mat labels, stats, centroids;
  int max_area = 0, max_area_num = 0, max_x, max_y, max_h, max_w;
  int n = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 4);
  for(int i = 0; i < n; i++){
    /** 最初の領域は画像の外枠なので無視 **/
    if(i == 0) continue;
    /** stats変数からラベル領域の矩形情報を取得 **/
    int *param = stats.ptr<int>(i);
    int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
    int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
    int w = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
    int h = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
    int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
    if(debug) cout << "(x, y, w, h) = " << x << ", " << y << ", " << w << ", " << h << "  (area) = " << area << endl;
    if(area > max_area){
      max_area = area;
      max_area_num = i;
      max_x = x;
      max_y = y;
      max_w = w;
      max_h = h;
    }
  }
  /*** 最大領域を持つものを得る ***/
  if(debug) cout << "最大(x, y, w, h) = " << max_x << ", " << max_y << ", " << max_w << ", " << max_h << endl;
  if(debug) cout << "最大面積 : " << max_area << endl;
  /*** 選別する ***/
  if(max_h == 0) return;
  /* 比率 */
  // double rate = (double)max_w / max_h;
  // if(rate >= 1.5 || rate < 1.0) return;
  /* コンテナの面積の2分の1以上の面積だったら弾く */
  // if(max_area > rect[0].width * rect[0].height / 2 || max_area < 500) return;
  /** ラベル部分のみの画像を作成 **/
  // label_img = gray(Rect(rect[0].x + max_x, rect[0].y + max_y + (max_h/2), w, max_h/2)).clone();
  label_img = gray(Rect(rect[0].x + max_x, rect[0].y + max_y, max_w, max_h)).clone();
  // label_img = binary(Rect(x, y, w, h)).clone();
  // cv::imshow("label", label_img);
  // label_img = binary(Rect(max_x + (max_w / 3), max_y + (max_h * 2 / 5), (max_w / 3), (max_h * 3 / 5))).clone(); //特徴点のときに重宝
  if(debug) cv::imshow("label", label_img);
  if(debug) cout << "(label_img_x, label_img_y) = " << label_img.cols << ", " << label_img.rows << endl;
}

/** ボツ **/
char labelTempleteMatching(const cv::Mat &img, prop::ptree &pt)
{
  int debug = 0;
  /*** パラメータ ***/
  int threshold = 10, ksize = 5, resize_h = 200;
  if(auto v = pt.get_optional<int>("CharacterParameter.BinaryThreshold")) threshold = v.get();
  if(auto v = pt.get_optional<int>("CharacterParameter.Ksize")) ksize = v.get();
  if(auto v = pt.get_optional<int>("CharacterParameter.ResizeH")) resize_h = v.get();
  /*** テンプレート画像の読み込み ***/
  Mat temp_a, temp_b, temp_c;
  temp_a =  imread("/mnt/hgfs/labo/address_label/delivery_address_a_200h.jpg", 0);
  temp_b =  imread("/mnt/hgfs/labo/address_label/delivery_address_b_200h.jpg", 0);
  temp_c =  imread("/mnt/hgfs/labo/address_label/delivery_address_c_200h.jpg", 0);

  /** 2値化 **/
  cv::Mat binary;
  cv::adaptiveThreshold(img, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, ksize, threshold);

  /*** img画像のサイズを変更 ***/
  cv::Mat resize;
  double mag;
  double mag_col = (double)temp_a.cols/(double)img.cols;
  double mag_row = (double)temp_a.rows/(double)img.rows;
  if(mag_col < mag_row) mag = mag_row;
  else mag = mag_col;
  cv::resize(binary, resize, Size(), mag, mag, INTER_LINEAR);
  cv::imshow("resize img", resize);

  /** 傾きを変えてテンプレートマッチング **/
  int min_angle = -10, max_angle = 10, step_angle = 1;
  cv::Point2i center(resize.cols/2, resize.rows/2); //中心
  double min_result_a, min_result_b, min_result_c;
  for(int angle = min_angle; angle <= max_angle; angle += step_angle){
    /** 画像回転 **/
    cv::Mat dst;
    cv::Mat change = cv::getRotationMatrix2D(center, angle, 1); //回転&拡大縮小
    cv::warpAffine(resize, dst, change, resize.size(), cv::INTER_CUBIC,cv::BORDER_CONSTANT); //画像の変換(アフィン変換)
    /** テンプレートマッチング **/
    cv::Mat result(1, 1, CV_32FC1);
    matchTemplate(dst, temp_a, result, TM_CCOEFF_NORMED);
    min_result_a = result.at<uchar>(0, 0);
    matchTemplate(dst, temp_b, result, TM_CCOEFF_NORMED);
    min_result_b = result.at<uchar>(0, 0);
    matchTemplate(dst, temp_c, result, TM_CCOEFF_NORMED);
    min_result_c = result.at<uchar>(0, 0);
  }

  cout << "aの類似度 ? = " << min_result_a << endl;
  cout << "bの類似度 ? = " << min_result_b << endl;
  cout << "cの類似度 ? = " << min_result_c << endl;

  /*** マッチングしてみる ***/ //だめです
  // cv::Mat result(1, 1, CV_32FC1);
  // matchTemplate(resize_img, temp_a, result, TM_CCOEFF_NORMED);
  // cout << "aの類似度 ? = " << +result.at<uchar>(0, 0) << endl;
  // matchTemplate(resize_img, temp_b, result, TM_CCOEFF_NORMED);
  // cout << "bの類似度 ? = " << +result.at<uchar>(0, 0) << endl;
  // matchTemplate(resize_img, temp_c, result, TM_CCOEFF_NORMED);
  // cout << "cの類似度 ? = " << +result.at<uchar>(0, 0) << endl;

  char result_char = 'n'; 
  if(min_result_a <= min_result_b && min_result_a <= min_result_c) result_char = 'a';
  else if(min_result_b < min_result_a && min_result_b <= min_result_c) result_char = 'b';
  else if(min_result_c < min_result_a && min_result_c < min_result_b) result_char = 'c';

  return result_char;
}

/*** 特徴点マッチング(AKAZE) ***/ //検出の精度は良いが動作が重い(時間がかかる)
char labelMatching(const cv::Mat &img, prop::ptree &pt)
{
  int debug;
  if(auto v = pt.get_optional<int>("Parameter.LabelMatchDebug")) debug = v.get();
  auto main_start = std::chrono::system_clock::now();
  /*** テンプレート画像の読み込み ***/
  // Mat temp_a, temp_b, temp_c;
  /** 良き **/
  // temp_a =  imread("/mnt/hgfs/labo/address_label/delivery_address_a.jpg", 0);
  // temp_b =  imread("/mnt/hgfs/labo/address_label/delivery_address_b.jpg", 0);
  // temp_c =  imread("/mnt/hgfs/labo/address_label/delivery_address_c.jpg", 0);

  /** ダメ(しかし上の4分の１まで処理時間短縮) **/
  // temp_a =  imread("/mnt/hgfs/labo/address_label/delivery_address_a_mini.jpg", 0);
  // temp_b =  imread("/mnt/hgfs/labo/address_label/delivery_address_b_mini.jpg", 0);
  // temp_c =  imread("/mnt/hgfs/labo/address_label/delivery_address_c_mini.jpg", 0);

  /*** かなり良い(実行時間200ミリ秒前後) ***/ //小さいため特徴点見つけづらい
  // temp_a =  imread("/mnt/hgfs/labo/address_label/delivery_address_a_test.jpg", 0);
  // temp_b =  imread("/mnt/hgfs/labo/address_label/delivery_address_b_test.jpg", 0);
  // temp_c =  imread("/mnt/hgfs/labo/address_label/delivery_address_c_test.jpg", 0);

  /*** かなり良い(実行時間300ミリ秒前後) ***/
  // temp_a =  imread("/mnt/hgfs/labo/address_label/delivery_address_a_test2.jpg", 0);
  // temp_b =  imread("/mnt/hgfs/labo/address_label/delivery_address_b_test2.jpg", 0);
  // temp_c =  imread("/mnt/hgfs/labo/address_label/delivery_address_c_test2.jpg", 0);

  /*** 文字部分のみ(小さめ:縦200ピクセル) ***/
  // temp_a =  imread("/mnt/hgfs/labo/address_label/delivery_address_a_test3.jpg", 0);
  // temp_b =  imread("/mnt/hgfs/labo/address_label/delivery_address_b_test3.jpg", 0);
  // temp_c =  imread("/mnt/hgfs/labo/address_label/delivery_address_c_test3.jpg", 0);

  /*** ぼかしたやつ ***/
  // temp_a =  imread("/mnt/hgfs/labo/address_label/delivery_address_a_resize_2.jpg", 0);
  // temp_b =  imread("/mnt/hgfs/labo/address_label/delivery_address_b_resize_2.jpg", 0);
  // temp_c =  imread("/mnt/hgfs/labo/address_label/delivery_address_c_resize_2.jpg", 0);

   /*** パラメータ ***/
  int threshold = 10, ksize = 5;
  if(auto v = pt.get_optional<int>("CharacterParameter.BinaryThreshold")) threshold = v.get();
  if(auto v = pt.get_optional<int>("CharacterParameter.Ksize")) ksize = v.get();
  
  cv::Mat resize, binary, half;

  /* 下半分だけ */
  // half = cv::Mat(img, cv::Rect(img.cols / 4, img.rows * 3 / 7, img.cols / 2, img.rows * 4 / 7 - img.rows / 20));

  /* 2値化 */
  // cv::threshold(img, binary, threshold, 255, cv::THRESH_BINARY);
  cv::adaptiveThreshold(img, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, ksize, threshold);
  // binary = img.clone();

  /*** img画像のサイズを変更 ***/
  cv::Mat resize_img;
  double mag;
  double mag_col = (double)temp_a.cols/(double)img.cols;
  double mag_row = (double)temp_a.rows/(double)img.rows;
  if(debug) cout << mag_col << endl;
  if(debug) cout << mag_row << endl;
  if(mag_col < mag_row) mag = mag_row;
  else mag = mag_col;
  // cv::resize(img, resize_img, Size(), mag, mag, INTER_LINEAR);
  cv::resize(binary, resize_img, Size(), mag, mag, INTER_LINEAR);
  if(debug) cv::imshow("resize img", resize_img);
  /*** テンプレート画像のサイズをimgに合わせる ***/
  // if(debug)cout << "(img_x, img_y) = " << img.cols << ", " << img.rows << endl;
  // cv::resize(temp_a, temp_a, img.size());
  // cv::resize(temp_b, temp_b, img.size());
  // cv::resize(temp_c, temp_c, img.size());
  // if(debug) cv::imshow("Template_a", temp_a);
  // if(debug) cv::imshow("Template_b", temp_b);
  // if(debug) cv::imshow("Template_c", temp_c);

  /** AKAZE **/
  cv::Mat dst_label, dst_a, dst_b, dst_c;
  /** AKAZE検出器の生成 **/
  cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.001f);

  /** 特徴点(キーポイント)を格納する配列 **/
  vector<cv::KeyPoint> key_label, key_temp_a, key_temp_b, key_temp_c;

  /*** 特徴点の検出 ***/
  akaze->detect(resize_img, key_label);
  akaze->detect(temp_a, key_temp_a);
  akaze->detect(temp_b, key_temp_b);
  akaze->detect(temp_c, key_temp_c);

  if(debug){
    /*** 特徴点の場所を描画 ***/
    // # DrawMatchesFlags::DRAW_RICH_KEYPOINTS  キーポイントのサイズと方向を描く
    cv::drawKeypoints(resize_img, key_label, dst_label, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(temp_a, key_temp_a, dst_a, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(temp_b, key_temp_b, dst_b, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(temp_c, key_temp_c, dst_c, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("特徴点", dst_label);
    cv::imshow("特徴点a", dst_a);
    cv::imshow("特徴点b", dst_b);
    cv::imshow("特徴点c", dst_c);
  }

  /*** 特徴量記述の計算 ***/
  cv::Mat des_label, des_a, des_b, des_c;
  akaze->compute(resize_img, key_label, des_label);
  akaze->compute(temp_a, key_temp_a, des_a);
  akaze->compute(temp_b, key_temp_b, des_b);
  akaze->compute(temp_c, key_temp_c, des_c);

  /***  マッチングアルゴリズムの選択 ***/
  // /*** 総当り ***/
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");//BruteForce(総当り)
  /*** FLANNの時用 ***/
  // if(des_label.type()!=CV_32F) des_label.convertTo(des_label, CV_32F);
  // if(des_a.type()!=CV_32F) des_a.convertTo(des_a, CV_32F);
  // if(des_b.type()!=CV_32F) des_b.convertTo(des_b, CV_32F);
  // if(des_c.type()!=CV_32F) des_c.convertTo(des_c, CV_32F);

  /*** 特徴点マッチング ***/
  /** 普通のmatch **/
  cout << "特徴点マッチング" << endl;
  vector<cv::DMatch> match_a, match_b, match_c;
  matcher->match(des_label, des_a, match_a);
  matcher->match(des_label, des_b, match_b);
  matcher->match(des_label, des_c, match_c);
  // vector<vector<cv::DMatch>> match_a, match_b, match_c;
  // matcher->knnMatch(des_label, des_a, match_a, 1);
  // matcher->knnMatch(des_label, des_b, match_b, 1);
  // matcher->knnMatch(des_label, des_c, match_c, 1);

  /*** マッチングの結果を画像化する ***/
  cv::Mat res_a, res_b, res_c;
  cv::drawMatches(resize_img, key_label, temp_a, key_temp_a, match_a, res_a);
  cv::drawMatches(resize_img, key_label, temp_b, key_temp_b, match_b, res_b);
  cv::drawMatches(resize_img, key_label, temp_c, key_temp_c, match_c, res_c);
  if(1){
    if(res_a.rows > 720){
      cv::resize(res_a, res_a, cv::Size(), 0.5, 0.5);
      cv::resize(res_b, res_b, cv::Size(), 0.5, 0.5);
      cv::resize(res_c, res_c, cv::Size(), 0.5, 0.5);
    }
    cv::imshow("結果a", res_a);
    cv::imshow("結果b", res_b);
    cv::imshow("結果c", res_c);
    cout << "結果表示完了" << endl;
  }

  /*** 類似度計算 ***/
  double sim_a = 0, sim_b = 0, sim_c = 0;
  /* a */
  for(int i = 0, size = match_a.size(); i < size; i++){
    cv::DMatch dis = match_a[i];
    sim_a += dis.distance;
  }
  sim_a /= match_a.size();
  cout << "aとの類似度 : " << sim_a << endl;
  /* b */
  for(int i = 0, size = match_b.size(); i < size; i++){
    cv::DMatch dis = match_b[i];
    sim_b += dis.distance;
  }
  sim_b /= match_b.size();
  cout << "bとの類似度 : " << sim_b << endl;
  /* c */
  for(int i = 0, size = match_c.size(); i < size; i++){
    cv::DMatch dis = match_c[i];
    sim_c += dis.distance;
  }
  sim_c /= match_c.size();
  cout << "cとの類似度 : " << sim_c << endl;

  /** knn **/
  // vector<vector<cv::DMatch>> match_a, match_b, match_c;
  // matcher->knnMatch(des_label, des_a, match_a, 1);
  // matcher->knnMatch(des_label, des_b, match_b, 1);
  // matcher->knnMatch(des_label, des_c, match_c, 1);

  // /*** マッチングの結果を画像化する ***/
  // cv::Mat res_a, res_b, res_c;
  // cv::drawMatches(resize_img, key_label, temp_a, key_temp_a, match_a, res_a);
  // cv::drawMatches(resize_img, key_label, temp_b, key_temp_b, match_b, res_b);
  // cv::drawMatches(resize_img, key_label, temp_c, key_temp_c, match_c, res_c);
  // if(1){
  //   if(res_a.rows > 720){
  //     cv::resize(res_a, res_a, cv::Size(), 0.5, 0.5);
  //     cv::resize(res_b, res_b, cv::Size(), 0.5, 0.5);
  //     cv::resize(res_c, res_c, cv::Size(), 0.5, 0.5);
  //   }
  //   cv::imshow("結果a", res_a);
  //   cv::imshow("結果b", res_b);
  //   cv::imshow("結果c", res_c);
  //   cout << "結果表示完了" << endl;
  // }

  // /*** 類似度計算 ***/
  // double sim_a = 0, sim_b = 0, sim_c = 0;
  // /* a */
  // for(int i = 0, size = match_a.size(); i < size; i++){
  //   cv::DMatch dis = match_a[i][0];
  //   sim_a += dis.distance;
  // }
  // sim_a /= match_a.size();
  // cout << "aとの類似度 : " << sim_a << endl;
  // /* b */
  // for(int i = 0, size = match_b.size(); i < size; i++){
  //   cv::DMatch dis = match_b[i][0];
  //   sim_b += dis.distance;
  // }
  // sim_b /= match_b.size();
  // cout << "bとの類似度 : " << sim_b << endl;
  // /* c */
  // for(int i = 0, size = match_c.size(); i < size; i++){
  //   cv::DMatch dis = match_c[i][0];
  //   sim_c += dis.distance;
  // }
  // sim_c /= match_c.size();
  // cout << "cとの類似度 : " << sim_c << endl;


  char result_char = 'n'; 
  if(sim_a < sim_b && sim_a < sim_c) result_char = 'a';
  else if(sim_b < sim_a && sim_b < sim_c) result_char = 'b';
  else if(sim_c < sim_a && sim_c < sim_b) result_char = 'c';
  
  auto main_end = std::chrono::system_clock::now();
  auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(main_end - main_start).count();
  cout << "AKAZE matching time : " << msec <<" milli sec" << endl;

  return result_char;
}


/*** (白黒の)パターンで認識する ***/
char labelPattern(const cv::Mat &img, prop::ptree &pt)
{
  /*** パラメータ ***/
  int threshold = 10, ksize = 5, resize_h = 200;
  if(auto v = pt.get_optional<int>("CharacterParameter.BinaryThreshold")) threshold = v.get();
  if(auto v = pt.get_optional<int>("CharacterParameter.Ksize")) ksize = v.get();
  if(auto v = pt.get_optional<int>("CharacterParameter.ResizeH")) resize_h = v.get();
  
  cv::Mat resize, binary, half;

  /* 下半分&真ん中範囲だけ */
  half = cv::Mat(img, cv::Rect(img.cols / 4, img.rows * 3 / 7, img.cols / 2, img.rows * 4 / 7 - img.rows / 20));

  /* 2値化 */
  // cv::threshold(half, binary, threshold, 255, cv::THRESH_BINARY);
  cv::adaptiveThreshold(half, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, ksize, threshold);
  /*** ノイズ処理 ***/
  // cv::medianBlur(binary, binary, 3);
  cv::resize(binary, binary, cv::Size(), (double)resize_h/half.rows, (double)resize_h/half.rows);

  /** 探索 **/
  uchar pre_pixel = 255;
  // vector<int> black_num_hold(binary.rows);
  vector<int> black_num_hold(4);
  for(int y_size = binary.rows, y = y_size/10; y < y_size; y++){
    uchar *ptr_binary = binary.ptr<uchar>(y);
    int black_num = 0;
    for(int x = 0, x_size = binary.cols; x < x_size; x++){
      if(pre_pixel == 255 && ptr_binary[x] == 0) black_num++;
      pre_pixel = ptr_binary[x];
    }
    // black_num_hold[y] = black_num;
    if(black_num == 1) black_num_hold[1]++;
    else if(black_num == 2) black_num_hold[2]++;
    else if(black_num == 3) black_num_hold[3]++;
    else black_num_hold[0]++;
    cout << "black_num: " << black_num << endl;
    if(y == y_size/3){
      std::vector<int>::iterator iter = std::max_element(black_num_hold.begin(), black_num_hold.end());
      size_t index = std::distance(black_num_hold.begin(), iter);
      if(index == 1) return 'b';
      black_num_hold.clear();
    }
    if(y == y_size * 2 / 5) black_num_hold.clear();
    if(y == y_size * 3 / 5){
      std::vector<int>::iterator iter = std::max_element(black_num_hold.begin(), black_num_hold.end());
      size_t index = std::distance(black_num_hold.begin(), iter);
      if(index == 1) return 'c';
      black_num_hold.clear();
    }
  }

  cv::imshow("test1", binary);
  return 'a';
}

/*** 10フレームの画像でパブリッシュする文字を判定する ***/
void decideAboutPublishLabel(const char &text)
{
  switch (text){
    case 'a':
      g_modo[0]++; g_count++; break;
    case 'b':
      g_modo[1]++; g_count++; break;
    case 'c':
      g_modo[2]++; g_count++; break;
    default:
      g_modo[3]++; g_count++; break;
  }
  std::cout << "g_count: " << g_count << endl;
  if(g_count == 10){ //10個取得したら最頻値を求める
    vector<int>::iterator iter = max_element(g_modo.begin(), g_modo.end());
    size_t index = std::distance(g_modo.begin(), iter);
    char label_char;
    switch (index){
      case 0:
        label_char = 'a';
        break;
      case 1:
        label_char = 'b';
        break;
      case 2:
        label_char = 'c';
        break;
      default:
        label_char = 'n';
        break;
    }
    g_count = 0;
    for(int i = 0, size = g_modo.size(); i < size; i++){
      g_modo[i] = 0;
    }
    if(label_char != 'n'){
      // RCLCPP_INFO(this->get_logger(), "label_char = %c.", label_char);
      // /** パブリッシュする動きを記述 **/
      // Char label_msg;
      // label_msg.data = label_char;
      // pub_label_->publish(label_msg);
    }
    // string label_st(1, label_char);
    cout << "test: " << label_char << endl; 
    label_st_ = string(1, label_char);
  }
}

/*** メイン関数 ***/
int main(int argc, char** argv)
{
  int debug, debug2;
  cv::Mat hsv, label_img, green_binary_img;
  vector<cv::Rect> green_rects;
  char pub_label;

  /*** iniファイルの読み込み ***/
  prop::ptree pt;
  prop::read_ini("../cfg/container_parameter.ini", pt);

  int many, stop_flag = 0;
  if(auto v = pt.get_optional<int>("Parameter.ManyPhoto")) many = v.get();
  if(auto v = pt.get_optional<int>("Parameter.MainDebug")) debug = v.get();
  if(auto v = pt.get_optional<int>("Parameter.MainDebug2")) debug2 = v.get();

  /*** 複数枚か、単体かで分かれる ***/
  if(many == 0){ //単体
    if(argc < 2){
      std::cerr << "Error : Usage is... " << argv[0] << " [image path]" << std::endl;
      return -1;
    }
    cv::Mat src = cv::imread(argv[1], 1);
    if(src.empty()) return -1;
    
    cv::waitKey(0);
  }else{ //複数
    int num_img, start_page;
    string folder;
    if(auto v = pt.get_optional<int>("Parameter.NumPhoto")) num_img= v.get();
    if(auto v = pt.get_optional<int>("Parameter.PhotoStart")) start_page= v.get();
    if(auto v = pt.get_optional<string>("Parameter.Folder")) folder= v.get();
    for(int i = start_page; i <= num_img; i++){
      /** 時間計測 **/
      auto main_start = std::chrono::system_clock::now();
      /*** 探索画像の読み込み ***/
      std::ostringstream name;
      name << folder << std::setw(6) << std::setfill('0') << i << ".png"; //serwは桁指定、setfileは桁の空いている部分を何で埋めるか
      cout << name.str() << endl;
      cv::Mat src = cv::imread(name.str(), 1);
      if(src.empty()) {
        cout << "画像の読み込みに失敗しました" << endl;
        return -1;
      }

      green_rects.clear();
      label_img.release();
      /*** 緑コンテナの検出 ***/
      /** コンテナを色検出 **/
      if(debug) cout << "コンテナ検出開始" << endl;
      colorDitector(src, hsv, green_binary_img, pt); //testしたい関数
      /** ラベリングし、矩形の大きさと矩形比率の計算を行いより高精度で検出 **/
      if(debug) cout << "ラベリング開始" << endl;
      labeling(green_binary_img, green_rects, pt);
      /** 更にコーナー検出を行い精度を上げたい **/
      /** 矩形が複数あった場合 **/
      if(green_rects.size() > 1){ //緑コンテナが複数個
        selectOneRectangle(green_rects);
      }

      cv::Mat rectangle_img;
      cvtColor(hsv, rectangle_img, COLOR_HSV2BGR);
      // rectangle_img = src.clone();
      
      /** 矩形がなければこの先の処理はスキップ **/
      if(!green_rects.empty()){
        /*** 矩形描画 ***/
        for(auto &rect : green_rects)
          rectangle(rectangle_img, rect, cv::Scalar(0, 255, 0), 2);

        /*** 緑コンテナの矩形内のラベルの検出 ***/
        if(debug) cout << "ラベル検出開始" << endl;
        labelDetector(hsv, label_img, green_rects, green_binary_img, pt);

        if(!label_img.empty()){
          /*** ラベル画像とテンプレート画像を比較してみる ***//** 1 = a, 2 = b, 3 = c **/
          if(debug) cout << "マッチング開始" << endl;
          char text;
          // text = labelTempleteMatching(label_img, pt);
          text = labelMatching(label_img, pt);
          // text = labelPattern(label_img, pt);
          /*** 結果表示(+画像上に文字を出す) ***/    
          if(text == 'n' && debug2){
            cout << "検出失敗" << endl;
            cv::putText(rectangle_img, "n", cv::Point(green_rects[0].x, green_rects[0].y), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
          }else{
            cout << "検出ラベルは" << text << "です" << endl;
            string text_st(1, text);
            cv::putText(rectangle_img, text_st, cv::Point(green_rects[0].x, green_rects[0].y), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
          }
          /** ROS2用にテスト **/
          /*** 10フレームの画像でパブリッシュする文字を判定する ***/
          // string label_st = decideAboutPublishLabel(text);
          decideAboutPublishLabel(text);
        }
          /*** 戻り値(パブリッシュした?)文字を表示 ***/
          cv::putText(rectangle_img, label_st_, cv::Point(0, rectangle_img.rows-10), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(255, 255, 255), 5);
      }
        /** 時間計測 **/
      auto main_end = std::chrono::system_clock::now();
      auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(main_end - main_start).count();
      cout << "main time : " << msec <<" milli sec" << endl; 
      
      resize(rectangle_img, rectangle_img, Size(), 0.5, 0.5);
      cv::imshow("grect", rectangle_img);
      
      int key;
      if(stop_flag == 0) key = cv::waitKey(0);
      if(stop_flag == 1) key = cv::waitKey(50);
      if(key == 'q') break;
      else if(key == 'p') std::cout << name.str() << std::endl;
      else if(key == 'b') i = i - 2;
      else if(key == 'c'){
        std::ostringstream ss;
        ss << "/mnt/hgfs/labo/label/label_" << std::setw(6) << std::setfill('0') << i << ".png";
        cv::imwrite(ss.str(), src);
        cout << "撮影成功" << endl;
      }
      else if(key == 's'){
        if(stop_flag == 0) stop_flag = 1;
        else stop_flag = 0;  
      }
    }
  }
  return 0;
}