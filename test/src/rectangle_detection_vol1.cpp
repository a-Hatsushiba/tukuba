#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <algorithm>

using namespace std;
namespace prop = boost::property_tree;


void svSearch(const cv::Mat src){
  double sum_sat = 0, sum_val = 0;
  cv::Mat hsv;
  /*** 画像をBGR->HSVに変換 ***/
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  for(int y = 0; y < src.rows; y++){
    cv::Vec3b *ptr_hsv = hsv.ptr<cv::Vec3b>(y);
    for(int x = 0; x < src.cols; x++){
      sum_sat += ptr_hsv[x][1];
      sum_val += ptr_hsv[x][2];
    }
  }
  double ave_sat = sum_sat / (1280*720);
  double ave_val = sum_val / (1280*720);
  cout << ave_sat << ", " << ave_val << endl;
}


/*** 特定色の抽出を行う(緑or青) ***///できたよ
void colorDitector(const cv::Mat &src, cv::Mat &result_green, cv::Mat &result_blue, prop::ptree &pt)
{
  /*** iniファイルからパラメータを読み込み ***/
  int green_hue_min_, green_hue_max_;
  int green_sat_min_, green_sat_max_;
  int green_val_min_, green_val_max_;
  int blue_hue_min_, blue_hue_max_;
  int blue_sat_min_, blue_sat_max_;
  int blue_val_min_, blue_val_max_;
  int debug;
  if(auto v = pt.get_optional<int>("GreenParameter.HueMin")) green_hue_min_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.HueMax")) green_hue_max_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.SatMin")) green_sat_min_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.SatMax")) green_sat_max_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.ValMin")) green_val_min_ = v.get();
  if(auto v = pt.get_optional<int>("GreenParameter.ValMax")) green_val_max_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.HueMin")) blue_hue_min_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.HueMax")) blue_hue_max_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.SatMin")) blue_sat_min_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.SatMax")) blue_sat_max_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.ValMin")) blue_val_min_ = v.get();
  if(auto v = pt.get_optional<int>("BlueParameter.ValMax")) blue_val_max_ = v.get();
  if(auto v = pt.get_optional<int>("Parameter.Debug")) debug = v.get();

  /*** HSV変換用 ***/
  cv::Mat hsv;
  /*** 画像をBGR->HSVに変換 ***/
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  /*** 出力画像を0で初期化しておく ***/
  result_green = cv::Mat(src.size(), CV_8U, cv::Scalar(0));
  result_blue = cv::Mat(src.size(), CV_8U, cv::Scalar(0));

  /*** 明度(V)のヒストグラム平坦化 ***/
  // vector<cv::Mat> planes;
  // vector<cv::Mat> hsv_merge;
  vector<cv::Mat> planes(3);
  vector<cv::Mat> hsv_merge(3);
  cv::Mat merge;
  //3つのチャンネル(H, S, V)に分離
  cv::split(hsv, planes);
  //SとVのヒストグラム平坦化を行う
  // cv::equalizeHist(planes[1], planes[1]);
  // cv::equalizeHist(planes[2], planes[2]);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
  // clahe->apply(planes[1], planes[1]);//これを消したほうがよく検出できる
  clahe->apply(planes[2], planes[2]);
  //分離したHSVのチャンネルを合成
  // hsv_merge.push_back(planes[0]);
  // hsv_merge.push_back(planes[1]);
  // hsv_merge.push_back(planes[2]);
  hsv_merge[0] = planes[0];
  hsv_merge[1] = planes[1];
  hsv_merge[2] = planes[2];
  cv::merge(hsv_merge, hsv);
  cv::cvtColor(hsv, merge, cv::COLOR_HSV2BGR);
  if(debug) cv::imshow("hist", merge);

  /*** 下半分の範囲でしきい値内である画素を抽出 ***/
  for(int rows = src.rows, y = rows * 1 / 2 ; y < rows; y++){
    /*** 行の先頭のポインタを取得 ***/
    cv::Vec3b *ptr_hsv = hsv.ptr<cv::Vec3b>(y);
    uchar *ptr_res_green = result_green.ptr<uchar>(y);
    uchar *ptr_res_blue = result_blue.ptr<uchar>(y);
    for(int x = 0, cols = src.cols; x < cols; x++){
      /*** 緑コンと青コンを検出 ***/
      if((blue_hue_min_ <= ptr_hsv[x][0] && ptr_hsv[x][0] <= blue_hue_max_)
          && (blue_sat_min_ <= ptr_hsv[x][1]) //&& ptr_hsv[x][1] <= blue_sat_max_)
          && (blue_val_min_ <= ptr_hsv[x][2])){// && ptr_hsv[x][2] <= blue_val_max_)){
            ptr_res_blue[x] = 255;
          }
      else if((green_hue_min_ <= ptr_hsv[x][0] && ptr_hsv[x][0] <= green_hue_max_)
          && (green_sat_min_ <= ptr_hsv[x][1]) //&& ptr_hsv[x][1] <= green_sat_max_)
          && (green_val_min_ <= ptr_hsv[x][2])){ //&& ptr_hsv[x][2] <= green_val_max_)){
            ptr_res_green[x] = 255;
          }
    }
  }
  /*** 平均化フィルタ ***/
  // cv::blur(result_blue, result_blue, cv::Size(3, 3));
  // cv::blur(result_green, result_green, cv::Size(3, 3));
  
  /*** メディアンフィルタ(ノイズ処理) ***/
  // cv::medianBlur(result_blue, result_blue, 3);
  // cv::medianBlur(result_green, result_green, 3);

  /*** 2回縮小膨張してみる(ノイズ処理＆細かい連結を切る) -> 不採用 ***/
  // erode(result_green, result_green, cv::noArray(), cv::Point(-1, -1), 3);
  // dilate(result_green, result_green, cv::noArray(), cv::Point(-1, -1), 3);
  // erode(result_blue, result_blue, cv::noArray(), cv::Point(-1, -1), 3);
  // dilate(result_blue, result_blue, cv::noArray(), cv::Point(-1, -1), 3);

  if(debug){
    cv::namedWindow("green_extract", cv::WINDOW_NORMAL);
    cv::namedWindow("blue_extract", cv::WINDOW_NORMAL);
    cv::imshow("green_extract", result_green);
    cv::imshow("blue_extract", result_blue);
  }
}

void labeling(const cv::Mat &result_green, const cv::Mat &result_blue, vector<cv::Rect> &green_rect, vector<cv::Rect> &blue_rect, prop::ptree &pt)
{
  cv::Mat labels, stats, centroids;
  int rows = result_green.rows;
  int rows_start = rows * 1 / 2;
  int rows_range = rows - rows_start;
  int write_to_file;
  int h_size_min_, w_size_min_, h_size_max_, w_size_max_;
  double rate_min_, rate_max_, g_area_per_, b_area_per_;
  /*** 新用 ***/
  int h_size_min1_, w_size_min1_, h_size_max1_, w_size_max1_;
  int h_size_min2_, w_size_min2_, h_size_max2_, w_size_max2_;
  // int w_size_min3_, w_size_max3_;
  if(auto v = pt.get_optional<double>("Parameter.HSizeMin1")) h_size_min1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMin1")) w_size_min1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.HSizeMax1")) h_size_max1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMax1")) w_size_max1_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.HSizeMin2")) h_size_min2_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMin2")) w_size_min2_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.HSizeMax2")) h_size_max2_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMax2")) w_size_max2_ = v.get();
  // if(auto v = pt.get_optional<double>("Parameter.WSizeMin3")) w_size_min3_ = v.get();
  // if(auto v = pt.get_optional<double>("Parameter.WSizeMax3")) w_size_max3_ = v.get();

  if(auto v = pt.get_optional<double>("Parameter.HSizeMin")) h_size_min_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMin")) w_size_min_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.HSizeMax")) h_size_max_ = v.get();
  if(auto v = pt.get_optional<double>("Parameter.WSizeMax")) w_size_max_ = v.get();
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
    // else if(y >= rows_start + rows_range * 2 / 3 && y < rows){
    //     cout << "green[3] = " << endl;
    //     cout << "green height = " << h << endl;
    //     cout << "green width = " << w << endl;
    //   if((w >= w_size_min3_ && w <= w_size_max3_) && (rate_min_ <= ((double)w / h) && ((double)w / h) <= rate_max_) && (area >= h * w *g_area_per_)){
    //     green_rect.push_back(cv::Rect(x, y, w, h));
    //     // cout << "green height = " << h << endl;
    //     // cout << "green width = " << w << endl;
    //     if(write_to_file){
    //       ofstream ofs("./container_size.txt", ios::app);
    //       ofs << h  << " " << w << endl;
    //     }
    //   }
    // }

    /*** 旧 ***/
    // if(((h > h_size_min_ && h < h_size_max_) && (w > w_size_min_ && w < w_size_max_)) && (rate_min_ < ((double)w / h) && ((double)w / h) < rate_max_) && (area >= h * w *g_area_per_)){
    //   green_rect.push_back(cv::Rect(x, y, w, h));
    //   // cout << "green height = " << h << endl;
    //   // cout << "green width = " << w << endl;
    //   if(write_to_file){
    //     ofstream ofs("./container_size.txt", ios::app);
    //     ofs << h  << " " << w << endl;
    //   }
    // }
  }
  /*** 青コン4近傍でラベリング ***/
  n = cv::connectedComponentsWithStats(result_blue, labels, stats, centroids, 4);
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
      if(((h > h_size_min1_ && h < h_size_max1_) && (w > w_size_min1_ && w < w_size_max1_)) && (rate_min_ <= ((double)w / h) && ((double)w / h) <= rate_max_) && (area >= h * w *b_area_per_)){
        blue_rect.push_back(cv::Rect(x, y, w, h));
        // cout << "blue height = " << h << endl;
        // cout << "blue width = " << w << endl;
        if(write_to_file){
          ofstream ofs("./container_size.txt", ios::app);
          ofs << h  << " " << w << endl;
        }
      }
    }else if(y >= rows_start + rows_range / 2 && y < rows){
        // cout << "blue[2]" << endl;
        // cout << "blue height = " << h << endl;
        // cout << "blue width = " << w << endl;
      if(((h > h_size_min2_ && h < h_size_max2_) && (w > w_size_min2_ && w < w_size_max2_)) && (area >= h * w *b_area_per_)){
        blue_rect.push_back(cv::Rect(x, y, w, h));
        // cout << "blue height = " << h << endl;
        // cout << "blue width = " << w << endl;
        if(write_to_file){
          ofstream ofs("./container_size.txt", ios::app);
          ofs << h  << " " << w << endl;
        }
      }
    }
    // else if(y >= rows_start + rows_range * 2 / 3 && y < rows){
    //     cout << "blue[3]" << endl;
    //     cout << "blue height = " << h << endl;
    //     cout << "blue width = " << w << endl;
    //   if((w >= w_size_min3_ && w <= w_size_max3_) && (rate_min_ <= ((double)w / h) && ((double)w / h) <= rate_max_) && (area >= h * w *b_area_per_)){
    //     blue_rect.push_back(cv::Rect(x, y, w, h));
    //     // cout << "blue height = " << h << endl;
    //     // cout << "blue width = " << w << endl;
    //     if(write_to_file){
    //       ofstream ofs("./container_size.txt", ios::app);
    //       ofs << h  << " " << w << endl;
    //     }
    //   }
    // }

    /*** 旧 ***/
    // if(((h > h_size_min_ && h < h_size_max_) && (w > w_size_min_ && w < w_size_max_)) && (rate_min_ <= ((double)w / h) && ((double)w / h) <= rate_max_) && (area >= h * w *b_area_per_)){
    //   blue_rect.push_back(cv::Rect(x, y, w, h));
    //   // cout << "blue height = " << h << endl;
    //   // cout << "blue width = " << w << endl;
    //   if(write_to_file){
    //     ofstream ofs("./container_size.txt", ios::app);
    //     ofs << h  << " " << w << endl;
    //   }
    // }
  }
}

/*** 複数矩形が検出されたらのやつ ***/
/*** コーナー検出を行う -> やめた -> 参考資料からエッジ検出 -> 輪郭検出 -> 四角(or六角)形で近似やってみる***/
void cornerDetection(cv::Mat &src, cv::Mat &binary_img, const vector<cv::Rect> &rects, prop::ptree &pt)
{
  int debug;
  if(auto v = pt.get_optional<double>("Parameter.Debug")) debug = v.get();
  cv::Mat black_img = cv::Mat(src.size(), CV_8U, cv::Scalar(0));
  
  /*** 矩形内で白色の部分をblack_imgに移す ***/
  for(int i = 0, rect_size = rects.size(); i < rect_size; i++){
    for(int y = rects[i].y, y_end = rects[i].y + rects[i].height; y < y_end; y++){
      uchar *ptr_bin = binary_img.ptr<uchar>(y);
      uchar *ptr_black = black_img.ptr<uchar>(y);
      for(int x = rects[i].x, x_end = rects[i].x + rects[i].width; x < x_end; x++){
        if(ptr_bin[x] == 255){
          ptr_black[x] = 255;
        }
      }
    }
  }

  // /*** 平均化フィルタ ***/
  // cv::blur(black_img, black_img, cv::Size(5, 5));
  // /*** メディアンフィルタ(ノイズ処理) ***/
  // cv::medianBlur(black_img, black_img, 3);

  if(1) cv::imshow("black_img", black_img);
  
  /*** 参考資料から形状特徴の近似を行う ***/
  /** ここから **/
  /*** black_imgの縁をすべて0(黒)にする -> 今後の作業のため ***/
  for(int y = 0, y_size = black_img.rows; y < y_size; y++){
    uchar *ptr_black = black_img.ptr<uchar>(y);
    int x_size = black_img.cols;
    if(y == 0 || y == (y_size - 1)){
      for(int x = 0; x < x_size; x++){
        ptr_black[x] = 0;
      }
    }else{
      ptr_black[0] = 0;
      ptr_black[x_size - 1] = 0;
    }
  }
  
  cv::Mat edge_img = black_img.clone();

  /*** 輪郭検出 ***/
  cv::Mat dst = src.clone();
  vector<vector<cv::Point>> contours;
  // vector<cv::Vec4i> hierarchy;
  cv::findContours(edge_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);//cv::CHAIN_APPROX_TC89_L1);
  /** 描画 **/
  if(1){//(debug){
    cv::drawContours(src, contours, -1, cv::Scalar(100, 0, 100), 2);
    cout << "輪郭検出:ok" << endl;
  }

  vector<double> contour_areas;
  vector<int> rect_array(rects.size(), 0);
  double min_complexity;
  int min_complexity_num = 0;
  int flag = 0;
  if(!contours.empty()){
    for(int i = 0, size = contours.size(); i < size; i++){
      if(debug) cout << "contours.size() = " << size << endl;
      double area = cv::contourArea(contours[i]);
      if(area > 9500){
        /*** 輪郭形状の近似 ***/       
        double eps_val;
        if(auto v = pt.get_optional<double>("Parameter.EpsVal")) eps_val = v.get();
        double esp_g = eps_val * cv::arcLength(contours[i], true); //1に近づくほど厳しくなる
        cv::approxPolyDP(cv::Mat(contours[i]), contours[i], esp_g, true);
        if(debug) cout << "近似G:ok" << endl;
        if(debug) cout << "contour[" << i << "] = " << contours[i].size() << endl;

        if(4 <= contours[i].size() && contours[i].size() <= 6){
          cv::drawContours(src, vector<vector<cv::Point>>{contours[i]}, -1, cv::Scalar(0, 0, 255), 2);
          /*** 各頂点がどの矩形領域に属しているかを検出 ***/
          for(int vertex = 0, vertex_size = contours[i].size(); vertex < vertex_size; vertex++){
            for(int rect_num = 0, rect_size = rects.size(); rect_num < rect_size; rect_num++){
              if((rects[rect_num].x <= contours[i][vertex].x) && (contours[i][vertex].x <= rects[rect_num].x + rects[rect_num].width)
                && (rects[rect_num].y <= contours[i][vertex].y) && (contours[i][vertex].y <= rects[rect_num].y + rects[rect_num].height)){
                  rect_array[rect_num]++;
              }
            }
          }
        }

        /*** 輪郭が長方形(四角)に近いかどうか ***/


        /*** 複雑度(周囲長^2 / 面積)で求める ***/
        // double length = cv::arcLength(contours[i], true);
        // double complexity = length * length / area;
        // // cout << "複雑度(" << i << ") = " << complexity << "  面積 = " << area << endl;
        // cv::drawContours(dst, vector<vector<cv::Point>>{contours[i]}, -1, cv::Scalar(0, 0, 255), 2);
        // if(flag == 0){
        //   min_complexity = complexity;
        //   min_complexity_num = i;
        //   flag = 1;
        // }else{
        //   if(min_complexity > complexity){
        //     min_complexity = complexity;
        //     min_complexity_num = i;
        //   }
        // }
      }
    }
    // cv::drawContours(dst, vector<vector<cv::Point>>{contours[min_complexity_num]}, -1, cv::Scalar(0, 0, 255), 2);
  }
  /** ここまで **/
}

/*** 矩形が複数個検出されてしまったときの処理 ***/
void selectOneRectangle(vector<cv::Rect> &rects){
  /*** 床においてあるコンテナは他の検出矩形よりも下の位置にあるのではないかという過程に基づいたもの
   * (つまりyの値が最も大きいyをコンテナと認識) ***/
  /*** rectsの最後の要素を取り出せばOK ***/
  rects.erase(rects.begin(), rects.end() - 1);
}

int main(int argc, char **argv)
{
  /*** iniファイルの読み込み ***/
  prop::ptree pt;
  prop::read_ini("../cfg/container_parameter.ini", pt);

  int many, stop_flag = 0;
  if(auto v = pt.get_optional<int>("Parameter.ManyPhoto")) many = v.get();

  if(many == 0 && argc < 2){
    std::cerr << "Error : Usage is... " << argv[0] << " [image path]" << std::endl;
    return -1;
  }
  cv::Mat green_binary_img, blue_binary_img;
  vector<cv::Rect> green_rects, blue_rects;
  if(many == 0){
    cv::Mat src = cv::imread(argv[1], 1);
    if(src.empty()) return -1;
    cv::namedWindow("src", cv::WINDOW_NORMAL);
    cv::imshow("src", src);

    /*** コンテナを色検出 ***/
    colorDitector(src, green_binary_img, blue_binary_img, pt); //testしたい関数
    /*** ラベリングし、矩形の大きさと矩形比率の計算を行いより高精度で検出 ***/
    labeling(green_binary_img, blue_binary_img, green_rects, blue_rects, pt);
    /*** 更にコーナー検出を行い精度を上げたい ***/
    /*** 矩形が複数あった場合 ***/
    if(green_rects.size() > 1){ //緑コンテナが複数個
      // cornerDetection(src, green_binary_img, green_rects, pt);
      selectOneRectangle(green_rects);
    }
    if(blue_rects.size() > 1){ //青コンテナが複数個
      // cornerDetection(src, blue_binary_img, blue_rects, pt);
      selectOneRectangle(blue_rects);
    }

    for (auto &rect : green_rects)
      rectangle(src, rect, cv::Scalar(0, 255, 0), 2);
    for (auto &rect : blue_rects)
      rectangle(src, rect, cv::Scalar(255, 0, 0), 2);
    cv::namedWindow("container", cv::WINDOW_NORMAL);
    cv::imshow("container", src);
    cv::waitKey(0);
  }else{
    int num_img, start_page;
    string folder; 
    if(auto v = pt.get_optional<int>("Parameter.NumPhoto")) num_img= v.get();
    if(auto v = pt.get_optional<int>("Parameter.PhotoStart")) start_page= v.get();
    if(auto v = pt.get_optional<string>("Parameter.Folder")) folder= v.get();
    for(int i = start_page; i <= num_img; i++){
      std::ostringstream name;
      // name << "./" << folder << "/" << std::setw(6) << std::setfill('0') << i << ".png"; //serwは桁指定、setfileは桁の空いている部分を何で埋めるか
      name << "/mnt/hgfs/SSD/" << folder << "/" << std::setw(6) << std::setfill('0') << i << ".png"; //serwは桁指定、setfileは桁の空いている部分を何で埋めるか
      //cout << name.str() << endl;
      cv::Mat src = cv::imread(name.str(), 1);
      if(src.empty()) {
        cout << "画像の読み込みに失敗しました" << endl;
        return -1;
      }

      // svSearch(src); //svの平均値
      // cv::namedWindow("src", cv::WINDOW_NORMAL);
      //cv::imshow("src", src);
      green_rects.clear();
      blue_rects.clear();
      /*** 色情報でコンテナを検出 ***/
      colorDitector(src, green_binary_img, blue_binary_img, pt);
      /*** ラベリングし、矩形の大きさと矩形の比率でコンテナ認識精度を上げる ***/
      labeling(green_binary_img, blue_binary_img, green_rects, blue_rects, pt);
      /*** 矩形が複数あった場合 ***/
      if(green_rects.size() > 1){ //緑コンテナが複数個
        // cornerDetection(src, green_binary_img, green_rects, pt);
        selectOneRectangle(green_rects);
      }
      if(blue_rects.size() > 1){ //青コンテナが複数個
        // cornerDetection(src, blue_binary_img, blue_rects, pt);
        selectOneRectangle(blue_rects);
      }

      /*** 下半分を更に三等分したよ ***/
      int rows_start = src.rows * 1 / 2;
      int rows_range = src.rows * 1 / 2;
      // cv::line(src, cv::Point(0, rows_start), cv::Point(src.cols - 1, rows_start), cv::Scalar(221, 143, 232), 2);
      // cv::line(src, cv::Point(0, rows_start + rows_range / 3), cv::Point(src.cols - 1, rows_start + rows_range / 3), cv::Scalar(221, 143, 232), 2);
      // cv::line(src, cv::Point(0, rows_start + rows_range * 2 / 3), cv::Point(src.cols - 1, rows_start + rows_range * 2 / 3), cv::Scalar(221, 143, 232), 2);
      /*** 二等分ね ***/
      cv::line(src, cv::Point(0, rows_start), cv::Point(src.cols - 1, rows_start), cv::Scalar(221, 143, 232), 2);
      cv::line(src, cv::Point(0, rows_start + rows_range / 2), cv::Point(src.cols - 1, rows_start + rows_range / 2), cv::Scalar(221, 143, 232), 2);


      for (auto &rect : green_rects)
        rectangle(src, rect, cv::Scalar(0, 255, 0), 2);
      for (auto &rect : blue_rects)
        rectangle(src, rect, cv::Scalar(255, 0, 0), 2);
      cv::namedWindow("container", cv::WINDOW_NORMAL);
      cv::imshow("container", src);
      int key;
      if(stop_flag == 0) key = cv::waitKey(0);
      if(stop_flag == 1) key = cv::waitKey(100);
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