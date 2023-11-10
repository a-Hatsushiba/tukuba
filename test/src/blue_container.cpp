#include "camera_container.hpp"
#include "lidar_container.hpp"

#define RANGE 15
#define PIXEL 0.1
#define out_debug 0

struct mouseParam{
  int x;
  int y;
  int event;
  int flag;
};

void onWindowEvent(int event, int x, int y, int flags, void *userdata)
{
  prop::ptree pt;
  prop::read_ini("../cfg/lidar_container.ini", pt);
  double range, pixel_size;
  if(auto v = pt.get_optional<double>("Parameter.Pixel")) pixel_size = v.get();
  if(auto v = pt.get_optional<double>("Parameter.Range")) range = v.get();

  if(event == cv::EVENT_LBUTTONDOWN){
    /** カーソルの絶対座標を取得 **/
    double abs_x = -((range / pixel_size) - x) * pixel_size;
    double abs_y = ((range / pixel_size) - y) * pixel_size;
    if(out_debug) cout << "x: " << abs_x << " y: " << abs_y << endl;
    if(!out_debug) cout << abs_x << "," << abs_y << ",";
    double distance = sqrt((abs_x * abs_x) + (abs_y * abs_y));
    if(out_debug) cout << "distance: " << distance << endl;
    if(!out_debug) cout << distance << endl;
  }
  
}

cv::Point2d xyInfo(const cv::Rect &rect, const cv::Mat &img)
{
  // cout << __func__ << " start!" << endl;
  if(out_debug) cout << "rect(x, y) : " << rect.x << ", " << rect.y << endl;
  double l_x = NULL, l_y = NULL, l_y_min = NULL, l_y_max = NULL;

  for(int y = rect.y; y < (rect.y + rect.height); y++){ //rect範囲内を探索
    const cv::Vec3d *ptr_img = img.ptr<cv::Vec3d>(y);
    for(int x = rect.x; x < (rect.x + rect.width); x++){
      // cout << "img(x, y, r_xy): " << ptr_img[x][0] << ", " << ptr_img[x][1] << ", " << ptr_img[x][2] << endl;
      /* 点群情報のx,yが0の場合を除いて処理を行う */
      if(ptr_img[x][2] > 2){
        /* とりあえずl_y_minとl_y_maxに値を入れる */
        if(l_y_min == NULL || l_y_max == NULL){
          l_y_min = ptr_img[x][1];
          l_y_max = ptr_img[x][1];
          // cout << "max: " << l_y_max << ", min: " << l_y_min << endl;
        }
        /* 点群情報で最大・最小のy探し */
        if(l_y_min > ptr_img[x][1]) l_y_min = ptr_img[x][1];
        if(l_y_max < ptr_img[x][1]) l_y_max = ptr_img[x][1];

        /* とりあえずl_xに値を入れる */
        if(l_x == NULL){
          l_x = ptr_img[x][0];
        }
        /* 点群情報で最も近いx探し *//*<-これは問題があるのでだめです*/
        if(ptr_img[x][0] > 2 && l_x < ptr_img[x][0]) l_x = ptr_img[x][0];
      }      
    }
  }
  l_y = (l_y_max + l_y_min)/2;

  return cv::Point2d(l_x, l_y);
}

void pointXY(cv::Mat &img, const cv::Point2d &point, const cv::Point2d &camera_point)
{
  // cout << __func__ << " start!" << endl;
  /*** 1m間隔の円を引く ***/
  if(out_debug) cout << "(x, y) : " << point.x << ", " << point.y << endl;
  cv::Mat bgr;
  cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
  for(int i = RANGE/1; i > 0; i--){
    if(i%5 == 0) cv::circle(bgr, cv::Point(RANGE/PIXEL, RANGE/PIXEL), 1/PIXEL*1*i, cv::Scalar(100, 0, 0), 1, 4);
    else cv::circle(bgr, cv::Point(RANGE/PIXEL, RANGE/PIXEL), 1/PIXEL*1*i, cv::Scalar(0, 100, 0), 1, 4);
  }
  // bgr.at<cv::Vec3b>(img.rows - (img.rows/2 + point.y*10), img.cols/2 + point.x*10) = cv::Vec3b(0, 0, 255);
  cv::circle(bgr, cv::Point(bgr.cols/2 + point.x*10, bgr.rows - (bgr.rows/2 + point.y*10)), 1, cv::Scalar(0, 0, 255), -1);
  // cv::circle(bgr, cv::Point(bgr.cols/2 + camera_point.x*10, bgr.rows - (bgr.rows/2 + camera_point.y*10)), 1, cv::Scalar(0, 217, 255), -1);
  cv::circle(bgr, cv::Point(bgr.cols - (bgr.cols/2 + camera_point.y * 10), bgr.rows - (bgr.rows/2 + camera_point.x * 10)), 1, cv::Scalar(0, 217, 255), -1);


  /*** マウスイベントの発生 ***/
  cv::namedWindow("point", 1);
  mouseParam mouse_param;
  cv::setMouseCallback("point", onWindowEvent, &mouse_param);
  cv::imshow("point", bgr);
}

int main(int argc, char** argv)
{
  /*** 標準出力をファイルに書く ***/
  // std::ofstream ofstr("../container_height_param.csv");
  // std::streambuf* strbuf;
  // // 変更前の値を取得
  // strbuf = std::cout.rdbuf( ofstr.rdbuf() );

  container::LiDARContainer lidar_cont;
  container::CameraContainer camera_cont;
  string file_path, file_img_path;
  bool stop_flag = true;
  int num;
  if(argc == 2) num = stoi(argv[1]);
  else num = 0;
  for(;;num++){
    cv::Mat camera_img, lidar_img, xy_info, result_img;
    cv::Rect blue_rect;
    cv::Point2d container_posi; 
    if(lidar_cont.getFileName(num, file_path, file_img_path)){ //lidarの点群情報、カメラの画像の入ったファイルパスを取得
      if(camera_cont.run(file_img_path, camera_img, blue_rect)){
        lidar_cont.run(file_path, lidar_img, xy_info);
        /*** lidar_imgとxy_infoをcamera_imgと同じ大きさにする ***/
        cv::resize(lidar_img, lidar_img, cv::Size(), (double)camera_img.cols/lidar_img.cols, (double)camera_img.rows/lidar_img.rows);
        cv::resize(xy_info, xy_info, cv::Size(), (double)camera_img.cols/xy_info.cols, (double)camera_img.rows/xy_info.rows);
        /*** カメラ画像の矩形部分の情報を抜き取る ***/
        if(!blue_rect.empty()){ //カメラの青コンの情報を得られたら
          container_posi = xyInfo(blue_rect, xy_info);
          // cout << "(x, y) : " << container_posi.x << ", " << container_posi.y << endl;
          cv::Mat xy_img;
          lidar_cont.create2DImgXY(xy_img);
          pointXY(xy_img, container_posi, camera_cont.rel_pose_);
          /*** test start ***/
          vector<cv::Mat> channels(2);
          cv::split(xy_info, channels);
          cv::Mat container_info(channels[2], blue_rect);
          vector<cv::Rect> cont_depth_rect;
          lidar_cont.labelingDepth(container_info, cont_depth_rect);
          for (auto &rect : cont_depth_rect)
            rectangle(lidar_img, cv::Rect(blue_rect.x + rect.x, blue_rect.y + rect.y, rect.width, rect.height), cv::Scalar(255, 0, 255), 1);
          /*** test finish ***/
        }
        /*** カメラと距離画像を重ねる ***/
        result_img = camera_img.clone();
        // cv::addWeighted(lidar_img, 1, camera_img, 0.8, 0, result_img);
        // cv::line(result_img, cv::Point(result_img.cols/2, 0), cv::Point(result_img.cols/2, result_img.rows-1), cv::Scalar(0, 0, 0), 1);
        cv::rectangle(result_img, blue_rect, cv::Scalar(255, 0, 0), 2);
        cv::resize(result_img, result_img, cv::Size(), 0.5, 0.5);
        cv::imshow("result", result_img);
        /*** 画像表示時のキーの動き ***/
        int key;
        if(stop_flag) key = cv::waitKey(0);
        if(!stop_flag) key = cv::waitKey(10);
        if(key == 'q'){
          // 元に戻す
          // std::cout.rdbuf( strbuf );
          break;
        }
        else if(key == 'p') std::cout << file_path << std::endl;
        else if(key == 'b') num = num - 2;
        else if(key == 's'){
          if(stop_flag == 0) stop_flag = 1;
          else stop_flag = 0;
        }
      }
      else break;
    }
    else break;
  }
  // 元に戻す
  // std::cout.rdbuf( strbuf );
  return 0;
}