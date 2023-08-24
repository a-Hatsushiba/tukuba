#include "rs_monitor2023/monitor2023.h"
#include "qt_image_draw/qt_image_draw.hpp"
#include "rs_monitor2023/ui_monitor2023.h"

#include <cv_bridge/cv_bridge.h>

// #include "ros2_rs_miyauchi_waypoint/waypoint_propertie.hpp"
#include "cvt_functions/quaternion_to_euler.hpp"
// #include "filesystem/get_home_dir.hpp"
// /*** cpp.oファイルしか見つからない ***/
// #include "point_img_generator/point_img_generator.hpp"

Monitor2023::Monitor2023(rclcpp::Node::SharedPtr node, QWidget *parent)
: QMainWindow(parent), node_(node), ui_(new Ui::Monitor2023)
{
  /*** Uiの初期化 ***/
  initUi();

  /*** パラメータの初期化 ***/
  initParam();

  /** サブスクライバの初期化 **/
  initSubscriber();

  /** QTimerの初期化 **/
  initQtimer();
}

Monitor2023::~Monitor2023()
{
  //delete ui_;
  //*thisが保持しているリソースの所有権を放棄する。リソースを解放するのではなく、解放する責任を放棄する。
  ui_.release();
}

void Monitor2023::initUi()
{
  RCLCPP_INFO(node_->get_logger(), "Initialize Ui...");
  ui_->setupUi(this);
  RCLCPP_INFO(node_->get_logger(), "Complete. Ui was initialized.");
}

void Monitor2023::initParam()
{
  RCLCPP_INFO(node_->get_logger(), "Initialize parameters...");
  // map_rotate_trim_.setMapImg(cv::imread(rs::get_homedir() + node_->declare_parameter("monitor.map_img", "map/campus/campus_map.png")));
  // map_rotate_trim_.setGridSize(node_->declare_parameter("monitor.meter_per_pixel", 0.1));
  // map_rotate_trim_.setMapTrimSize(node_->declare_parameter("monitor.map_view_size", 50.0));
  depth_max_ = node_->declare_parameter("monitor.depth_max", 30.0);
  // log_filename_ = node_->declare_parameter("", "rs_log_2023");
  logger_is_active_ = std::make_shared<Bool>();
  mercury_run_is_active_ = std::make_shared<Bool>();
  path_planning_is_active_ = std::make_shared<Bool>();
  path_planning_task_num_ = std::make_shared<Int32>();
  path_following_is_active_ = std::make_shared<Bool>();
  waypoint_manager_is_active_ = std::make_shared<Bool>();
  joystick_is_connected_ = std::make_shared<Bool>();
  // road_sign_is_active = std::make_shared<Bool>();
  // road_sign_infos = std::make_shared<SignalInfoArray>();
  RCLCPP_INFO(node_->get_logger(), "Complete. Parameters were initialized.");
}

void Monitor2023::initSubscriber()
{
  using std::placeholders::_1;

  RCLCPP_INFO(node_->get_logger(), "Initialize Subscriber...");
  sub_lidar_h_ = node_->create_subscription<Empty>("/lidar/top_lidar_active", 10, std::bind(&Monitor2023::onHighLidarSubscribed, this, _1));
  sub_lidar_m_ = node_->create_subscription<Empty>("/lidar/middle_lidar_active", 10, std::bind(&Monitor2023::onMidLidarSubscribed, this, _1));
  sub_lidar_l_ = node_->create_subscription<Empty>("/lidar/bottom_lidar_active", 10, std::bind(&Monitor2023::onLowLidarSubscribed, this, _1));
  sub_camera_img_ = node_->create_subscription<Image>("/camera/image", 10, std::bind(&Monitor2023::onCameraImgSubscribed, this, _1));
  sub_sonar_range_ = node_->create_subscription<Float32MultiArray>("/sonar/ranges", 10, std::bind(&Monitor2023::onSonarRangeSubscribed, this, _1));
  sub_mercury_state_ = node_->create_subscription<RobotStateMsg>("/mercury/state", 10, std::bind(&Monitor2023::onMercuryStateSubscribed, this, _1));
  sub_mercury_run_ = node_->create_subscription<Bool>("/mercury_run/is_active", 10, std::bind(&Monitor2023::onMercuryRunSubscribed, this, _1));
  // sub_waypoint_ = node_->create_subscription<Waypoint>("/waypoint_manager/waypoint", 10, std::bind(&Monitor2023::onWaypointSubscribed, this, _1));
  sub_waypoint_manager_is_active_ = node_->create_subscription<Bool>("/waypoint_manager/is_active", 10, std::bind(&Monitor2023::onWaypointManagerActiveSubscribed, this, _1));
  sub_waypoint_file_ = node_->create_subscription<String>("/waypoint_manager/waypoint_file", 10, std::bind(&Monitor2023::onWaypointFileSubscribed, this, _1));
  sub_waypoint_img_ = node_->create_subscription<Image>("/waypoint_manager/image", 10, std::bind(&Monitor2023::onWaypointImgSubscribed, this, _1));
  sub_waypoint_index_num_ = node_->create_subscription<Int32>("/waypoint_manager/index", 10, std::bind(&Monitor2023::onWaypointIndexNumSubscribed, this, _1));  
  sub_logger_is_active_ = node_->create_subscription<Bool>("/logger/is_active", 10, std::bind(&Monitor2023::onLoggerActiveSubscribed, this, _1));
  sub_path_planning_target_odometry_ = node_->create_subscription<Odometry>("/path_planning/target_odom", 10, std::bind(&Monitor2023::onPathPlanningTargetOdometrySubscribed, this, _1));
  sub_path_planning_task_num_ = node_->create_subscription<Int32>("/path_planning/task_num", 10, std::bind(&Monitor2023::onPathPlanningTaskNumSubscribed, this, _1));
  sub_path_planning_is_active_ = node_->create_subscription<Bool>("/path_planning/is_active", 10, std::bind(&Monitor2023::onPathPlanningActiveSubscribed, this, _1));
  sub_path_following_is_active_ = node_->create_subscription<Bool>("/path_following/is_active", 10, std::bind(&Monitor2023::onPathFollowingActiveSubscribed, this, _1));
  sub_location_pose_ = node_->create_subscription<Pose>("/locator/corrected_pose", 10, std::bind(&Monitor2023::onLocationPoseSubscribed, this, _1));
  sub_location_scanmatting_img_ = node_->create_subscription<Image>("/locator/scanmatch_img", 10, std::bind(&Monitor2023::onLocationScanMattingImgSubscribed, this, _1));
  sub_obstacle_img_ = node_->create_subscription<Image>("/obstacle_detector/image", 10, std::bind(&Monitor2023::onObstacleImgSubscribed, this, _1));
  sub_depth_img_ = node_->create_subscription<Image>("/points_processor/depth_img", 10, std::bind(&Monitor2023::onDepthImgSubscribed, this, _1));
  sub_ref_img_ = node_->create_subscription<Image>("/points_processor/ref_img", 10, std::bind(&Monitor2023::onRefImgSubscribed, this, _1));
  sub_joustick_is_connected_ = node_->create_subscription<Bool>("/joy/is_connected", 10, std::bind(&Monitor2023::onJoystickConnectSubscribed, this, _1));
  sub_cmd_robot_vel_ = node_->create_subscription<Twist>("/mercury/cmd_robot_vel", 10, std::bind(&Monitor2023::onCmdRobotVelSubscribed, this, _1));
  sub_mileage_ = node_->create_subscription<Int32>("/mileage", 10, std::bind(&Monitor2023::onMileageSubscribed, this, _1));
  // sub_log_filename_ = node_->create_subscription<String>("/log/filename", 10, std::bind(&Monitor2023::onLogFilenameSubscribed, this, _1));
  sub_log_auto_ = node_->create_subscription<Int32>("/rs_logger/auto_save_cnt", 10, std::bind(&Monitor2023::onLogAutoSubscribed, this, _1));
  sub_log_manual_ = node_->create_subscription<Int32>("/rs_logger/manual_save_cnt", 10, std::bind(&Monitor2023::onLogManualSubscribed, this, _1));
  RCLCPP_INFO(node_->get_logger(), "Complete. Subscriber was initialized.");
}

void Monitor2023::initQtimer()
{
  /** センサ監視用ループ処理(100ms毎に更新) **/
  QTimer *timer1 = new QTimer(this);
  QObject::connect(timer1, &QTimer::timeout, this, &Monitor2023::updateGui);
  timer1->start(100);

  /** Monitor内のNodeStatusの値の更新(500ms毎に更新) **/
  QTimer *timer2 = new QTimer(this);
  QObject::connect(timer2, &QTimer::timeout, this, &Monitor2023::updateNodeStatus);
  timer2->start(500);
}

/** 3DliDAR(H)受信時のコールバック関数 **/
void Monitor2023::onHighLidarSubscribed(const Empty::SharedPtr empty)
{
  lidar_h_mutex.lock();
  lidar_h_is_received = true;
  lidar_h_mutex.unlock();
}

/** 3DliDAR(M)受信時のコールバック関数 **/
void Monitor2023::onMidLidarSubscribed(const Empty::SharedPtr empty)
{
  lidar_m_mutex.lock();
  lidar_m_is_received = true;
  lidar_m_mutex.unlock();
}

/** 2DliDAR(L)受信時のコールバック関数 **/
void Monitor2023::onLowLidarSubscribed(const Empty::SharedPtr empty)
{
  lidar_l_mutex.lock();
  lidar_l_is_received = true;
  lidar_l_mutex.unlock();
}

/** カメラ画像受信時のコールバック関数 **/
void Monitor2023::onCameraImgSubscribed(const Image::SharedPtr img)
{
  if(img->data.empty()) return;
  camera_mutex.lock();
  camera_img_ = img;
  camera_is_received = true;
  camera_mutex.unlock();
}

/** ソナーの値を受信した時のコールバック関数 **/
void Monitor2023::onSonarRangeSubscribed(const Float32MultiArray::SharedPtr range)
{
  if(range->data.empty()) return;
  sonar_range_mutex.lock();
  sonar_range_ = range;
  sonar_range_is_received = true;
  sonar_range_mutex.unlock();
}

/** Mercuryステータス(マニュアル、自律)受信時のコールバック関数 **/
void Monitor2023::onMercuryStateSubscribed(const RobotStateMsg::SharedPtr state)
{
  mercury_mutex.lock();
  mercury_state_ = state;
  mercury_is_received = true;
  mercury_mutex.unlock();
}

/** Mercury動作時のコールバック関数 **/
void Monitor2023::onMercuryRunSubscribed(const Bool::SharedPtr active)
{
  mercury_run_mutex.lock();
  mercury_run_is_active_ = active;
  mercury_run_is_received = true;
  mercury_run_mutex.unlock();
}


/** Waypointの状態を受信した時のコールバック関数 **/
void Monitor2023::onWaypointManagerActiveSubscribed(const Bool::SharedPtr active)
{
  waypoint_mutex.lock();
  waypoint_manager_is_active_ = active;
  waypoint_is_received = true;
  waypoint_mutex.unlock();
}

/** Waypointの画像を受信した時のコールバック関数 **/
void Monitor2023::onWaypointImgSubscribed(const Image::SharedPtr img)
{
  waypoint_mutex.lock();
  waypoint_img_ = img;
  waypoint_is_received = true;
  waypoint_mutex.unlock();
}

/** Waypoint受信時のコールバック関数 **/
// void Monitor2023::onWaypointSubscribed(const Waypoint::SharedPtr waypoint)
// {
//   waypoint_mutex.lock();
//   waypoint_ = waypoint;
//   waypoint_is_received = true;
//   waypoint_mutex.unlock();
// }

/** Waypointのファイル名を受信した時のコールバック関数 **/
void Monitor2023::onWaypointFileSubscribed(const String::SharedPtr filename)
{
  waypoint_mutex.lock();
  waypoint_file_ = filename;
  waypoint_is_received = true;
  waypoint_mutex.unlock();
}

void Monitor2023::onWaypointIndexNumSubscribed(const Int32::SharedPtr num)
{
  waypoint_mutex.lock();
  waypoint_index_num_ = num;
  waypoint_is_received = true;
  waypoint_mutex.unlock();
}

/** Logger動作時のコールバック関数 **/
void Monitor2023::onLoggerActiveSubscribed(const Bool::SharedPtr active)
{
  logger_mutex.lock();
  logger_is_active_ = active;
  logger_is_received = true;
  logger_mutex.unlock();
}

/** Path planning対象受信時のコールバック関数 **/
void Monitor2023::onPathPlanningTargetOdometrySubscribed(const Odometry::SharedPtr odometry)
{
  path_planning_mutex.lock();
  path_planning_target_odmetry_ = odometry;
  path_planning_is_received = true;
  path_planning_mutex.unlock();
}

void Monitor2023::onPathPlanningTaskNumSubscribed(const Int32::SharedPtr num)
{
  path_planning_mutex.lock();
  path_planning_task_num_ = num;
  path_planning_is_received = true;
  path_planning_mutex.unlock();
}

/** Path planning動作時のコールバック関数 **/
void Monitor2023::onPathPlanningActiveSubscribed(const Bool::SharedPtr active)
{
  path_planning_mutex.lock();
  path_planning_is_active_ = active;
  path_planning_is_received = true;
  path_planning_mutex.unlock();
}

/** Path following動作時のコールバック関数 **/
void Monitor2023::onPathFollowingActiveSubscribed(const Bool::SharedPtr active)
{
  path_following_mutex.lock();
  path_following_is_active_ = active;
  path_following_is_received = true;
  path_following_mutex.unlock();
}

/** 自己位置受信時のコールバック関数 **/
void Monitor2023::onLocationPoseSubscribed(const Pose::SharedPtr pose)
{
  location_mutex.lock();
  location_pose_ = pose;
  location_is_received =true;
  location_mutex.unlock();
}

/** Scan matching画像受信時のコールバック関数 **/
void Monitor2023::onLocationScanMattingImgSubscribed(const Image::SharedPtr img)
{
  if(img->data.empty()) return;
  location_mutex.lock();
  location_scanmatting_img_ = img;
  location_is_received =true;
  location_mutex.unlock();
}

/** 障害物画像受信時のコールバック関数 **/
void Monitor2023::onObstacleImgSubscribed(const Image::SharedPtr img)
{
  if(img->data.empty()) return;
  obstacle_mutex.lock();
  obstacle_img_ = img;
  obstacle_is_received = true;
  obstacle_mutex.unlock();
}

/** 距離画像受信時のコールバック関数 **/
void Monitor2023::onDepthImgSubscribed(const Image::SharedPtr img)
{
  if(img->data.empty()) return;
  points_processor_mutex.lock();
  depth_img_ = img;
  points_processor_is_received = true;
  points_processor_mutex.unlock();
}

/** 強度反射画像受信時のコールバック関数 **/
void Monitor2023::onRefImgSubscribed(const Image::SharedPtr img)
{
  if(img->data.empty()) return;
  points_processor_mutex.lock();
  ref_img_ = img;
  points_processor_is_received = true;
  points_processor_mutex.unlock();
}

/** Joystick接続時のコールバック関数 **/
void Monitor2023::onJoystickConnectSubscribed(const Bool::SharedPtr connect)
{
  joystick_mutex.lock();
  joystick_is_connected_ = connect;
  joystick_is_received = true;
  joystick_mutex.unlock();
}

/** mercury制御パラメータ受信時のコールバック関数 **/
void Monitor2023::onCmdRobotVelSubscribed(const Twist::SharedPtr vel)
{
  cmd_robot_vel_mutex.lock();
  cmd_robot_vel_ = vel;
  cmd_robot_vel_mutex.unlock();
}

/** 走行距離受信時のコールバック関数 **/
void Monitor2023::onMileageSubscribed(const Int32::SharedPtr mile)
{
  mileage_mutex.lock();
  mileage_ = mile;
  mileage_mutex.unlock();
}

/** Log情報受信時のコールバック関数 **/
// void Monitor2023::onLogFilenameSubscribed(const String::SharedPtr name)
// {
//   loginfo_mutex.lock();
//   log_filename_ = name;
//   loginfo_mutex.unlock();
// }

void Monitor2023::onLogAutoSubscribed(const Int32::SharedPtr num)
{
  loginfo_mutex.lock();
  log_auto_ = num;
  loginfo_mutex.unlock();
}

void Monitor2023::onLogManualSubscribed(const Int32::SharedPtr num)
{
  loginfo_mutex.lock();
  log_manual_ = num;
  loginfo_mutex.unlock();
}

/** Monitor(GUI)のNodeStatusラベルの更新 **/
/** 信号を受信できていれば緑、できていなければ赤 **/
void Monitor2023::updateNodeStatus()
{
  /** 3DLiDAR(H) **/
  lidar_h_mutex.lock();
  if(lidar_h_is_received == true){
    ui_->light_3DLiDAR_H->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_3DLiDAR_H->setStyleSheet("background-color:red;");
  }
  lidar_h_is_received = false;
  lidar_h_mutex.unlock();

  /** 3DLiDAR(M) **/
  lidar_m_mutex.lock();
  if(lidar_m_is_received == true){
    ui_->light_3DLiDAR_M->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_3DLiDAR_M->setStyleSheet("background-color:red;");
  }
  lidar_m_is_received = false;
  lidar_m_mutex.unlock();

  /** 2DLiDAR(L) **/
  lidar_l_mutex.lock();
  if(lidar_l_is_received == true){
    ui_->light_2DLiDAR_L->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_2DLiDAR_L->setStyleSheet("background-color:red;");
  }
  lidar_l_is_received = false;
  lidar_l_mutex.unlock();

  /** Camera(F) **/
  camera_mutex.lock();
  if(camera_is_received == true){
    ui_->light_camera_status->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_camera_status->setStyleSheet("background-color:red;");
  }
  camera_is_received = false;
  camera_mutex.unlock();

  /*** Soner ***/
  sonar_range_mutex.lock();
  if(sonar_range_is_received == true){
    ui_->light_sonar->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_sonar->setStyleSheet("background-color:red;");
  }
  sonar_range_is_received = false;
  sonar_range_mutex.unlock();

  /** Mercury **/
  mercury_mutex.lock();
  if(mercury_is_received == true){
    ui_->light_Mercury->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_Mercury->setStyleSheet("background-color:red;");
  }
  mercury_is_received = false;
  mercury_mutex.unlock();

  /** MercuryRun **/
  mercury_run_mutex.lock();
  if(mercury_run_is_received == true){
    ui_->light_Mercury_run->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_Mercury_run->setStyleSheet("background-color:red;");
  }
  mercury_run_is_received = false;
  mercury_run_mutex.unlock();

  /** Waypoint **/
  waypoint_mutex.lock();
  if(waypoint_is_received == true){
    ui_->light_waypoint->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_waypoint->setStyleSheet("background-color:red;");
  }
  waypoint_is_received = false;
  waypoint_mutex.unlock();

  /** Logger **/
  logger_mutex.lock();
  if(logger_is_received == true){
    ui_->light_logger->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_logger->setStyleSheet("background-color:red;");
  }
  logger_is_received = false;
  logger_mutex.unlock();

  /** PathPlanning **/
  path_planning_mutex.lock();
  if(path_planning_is_received == true){
    ui_->light_path_planning->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_path_planning->setStyleSheet("background-color:red;");
  }
  path_planning_is_received = false;
  path_planning_mutex.unlock();

  /** PathFollowing **/
  path_following_mutex.lock();
  if(path_following_is_received == true){
    ui_->light_path_following->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_path_following->setStyleSheet("background-color:red;");
  }
  path_following_is_received = false;
  path_following_mutex.unlock();

  /** Locator **/
  location_mutex.lock();
  if(location_is_received == true){
    ui_->light_locator->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_locator->setStyleSheet("background-color:red;");
  }
  location_is_received = false;
  location_mutex.unlock();

  /** Obstacle **/
  obstacle_mutex.lock();
  if(obstacle_is_received == true){
    ui_->light_obstacle->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_obstacle->setStyleSheet("background-color:red;");
  }
  obstacle_is_received = false;
  obstacle_mutex.unlock();

  /** PointsProc **/
  points_processor_mutex.lock();
  if(points_processor_is_received == true){
    ui_->light_points_proc->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_points_proc->setStyleSheet("background-color:red;");
  }
  points_processor_is_received = false;
  points_processor_mutex.unlock();

  /** Joystick **/
  joystick_mutex.lock();
  if(joystick_is_received == true){
    ui_->light_joystick->setStyleSheet("background-color:lime;");
  }
  else{
    ui_->light_joystick->setStyleSheet("background-color:red;");
  }
  joystick_is_received = false;
  joystick_mutex.unlock();

  /** RoadSign **/
  /** Container **/
  /** Traffic light **/
}

void Monitor2023::updateGui() //setText()でstring型を使うときにはstr()ではなく、c_str()またはQString型に直して
{
  char str[64];
  /* ----------------------- Mercury statusの更新 ------------------------ */ //途中
  mercury_mutex.lock();
  auto mercury_state = mercury_state_;
  mercury_mutex.unlock();
  if(mercury_state.get()){
    /** バッテリーの電圧計(Battery) **/ //snprintfのほうがostringstreamより速い
    std::snprintf(str, sizeof(str), "%.1f V", mercury_state->battery);
    ui_->label_bolt->setText(str);
    int ratio = 100 * (mercury_state->battery - 21.0) / (29.0 - 21.0);
    ui_->progressBar_battery->setValue(ratio < 0 ? 0: ratio);
    if(ratio > 60){
      ui_->progressBar_battery->setStyleSheet("selection-background-color:lime; background-color:rgb(243, 243, 243);");// 緑
    }else if(30 < ratio && ratio <= 60){
      ui_->progressBar_battery->setStyleSheet("selection-background-color:orange; background-color:rgb(243, 243, 243);"); //オレンジ
    }else{
      ui_->progressBar_battery->setStyleSheet("selection-background-color:red; background-color:rgb(243, 243, 243);");//赤
    }
    //ココらへんである値以下になったらポップアップを出すようにしたい
    

    /** 内部温度(Temp) **/
    ratio = mercury_state->temperature;
    ui_->progressBar_temp->setValue(ratio);
    if(0 <= ratio && ratio < 60){
      ui_->progressBar_temp->setStyleSheet("selection-background-color:lime; background-color:rgb(243, 243, 243);");// 緑
    }else if(60 <= ratio && ratio < 90){
      ui_->progressBar_temp->setStyleSheet("selection-background-color:orange; background-color:rgb(243, 243, 243);"); //オレンジ
    }else{
      ui_->progressBar_temp->setStyleSheet("selection-background-color:red; background-color:rgb(243, 243, 243);");//赤
    }

    /** CPU温度(CPU Temp) **/
    ratio = mercury_state->cpu_temperature;
    ui_->progressBar_temp_CPU->setValue(ratio);
    if(0 <= ratio && ratio < 60){
      ui_->progressBar_temp_CPU->setStyleSheet("selection-background-color:lime; background-color:rgb(243, 243, 243);");// 緑
    }else if(60 <= ratio && ratio < 90){
      ui_->progressBar_temp_CPU->setStyleSheet("selection-background-color:orange; background-color:rgb(243, 243, 243);"); //オレンジ
    }else{
      ui_->progressBar_temp_CPU->setStyleSheet("selection-background-color:red; background-color:rgb(243, 243, 243);");//赤
    }

    /** ホイールの回転数計 **/
    ui_->rpm_front_left->setText(std::to_string(mercury_state->rpm[0]).c_str());
    ui_->rpm_front_right->setText(std::to_string(mercury_state->rpm[2]).c_str());
    ui_->rpm_rear_left->setText(std::to_string(mercury_state->rpm[1]).c_str());
    ui_->rpm_rear_right->setText(std::to_string(mercury_state->rpm[3]).c_str());

    /** IMU **/
    std::snprintf(str, sizeof(str), "%.2f", mercury_state->imu.linear_acceleration.x);
    ui_->label_accele_x->setText(str);
    std::snprintf(str, sizeof(str), "%.2f", mercury_state->imu.linear_acceleration.y);
    ui_->label_accele_y->setText(str);
    std::snprintf(str, sizeof(str), "%.2f", mercury_state->imu.linear_acceleration.z);
    ui_->label_accele_z->setText(str);
    std::snprintf(str, sizeof(str), "%.2f", mercury_state->imu.angular_velocity.x);
    ui_->label_gyro_x->setText(str);
    std::snprintf(str, sizeof(str), "%.2f", mercury_state->imu.angular_velocity.y);
    ui_->label_gyro_y->setText(str);
    std::snprintf(str, sizeof(str), "%.2f", mercury_state->imu.angular_velocity.z);
    ui_->label_gyro_z->setText(str);

    double roll, pitch, yaw;
    project_ryusei::quaternionToEulerAngle(mercury_state->imu.orientation, &roll, &pitch, &yaw);
    std::snprintf(str, sizeof(str), "%.2f", roll);
    ui_->label_angle_x->setText(str);
    std::snprintf(str, sizeof(str), "%.2f", pitch);
    ui_->label_angle_y->setText(str);
    std::snprintf(str, sizeof(str), "%.2f", yaw);
    ui_->label_angle_z->setText(str);
  }

  /* ----------------------- Mercury runの更新 ------------------------ */
  mercury_run_mutex.lock();
  if(mercury_run_is_active_->data == true){
    ui_->light_Mercury_run->setText("A");
  }else{
    ui_->light_Mercury_run->setText("-");
  }
  mercury_run_mutex.unlock();

  /* ----------------------- Sonarの更新 ------------------------ */
  sonar_range_mutex.lock();
  auto sonar_range = sonar_range_;
  sonar_range_ = nullptr;
  sonar_range_mutex.unlock();
  if(sonar_range.get()){
    std::snprintf(str, sizeof(str), "%.2f", sonar_range->data[5]);
    ui_->Sonar_front_left->setText(str); // 左前
    std::snprintf(str, sizeof(str), "%.2f", sonar_range->data[0]);
    ui_->Sonar_front_center->setText(str); // 前
    std::snprintf(str, sizeof(str), "%.2f", sonar_range->data[3]);
    ui_->Sonar_front_right->setText(str); // 右前
    std::snprintf(str, sizeof(str), "%.2f", sonar_range->data[6]);
    ui_->Sonar_left->setText(str); // 左
    std::snprintf(str, sizeof(str), "%.2f", sonar_range->data[2]);
    ui_->Sonar_right->setText(str); // 右
    std::snprintf(str, sizeof(str), "%.2f", sonar_range->data[4]);
    ui_->Sonar_rear_left->setText(str); // 左後
    std::snprintf(str, sizeof(str), "%.2f", sonar_range->data[1]);
    ui_->Sonar_rear_right->setText(str); // 右後
  }

  /* ----------------------- Joystickの更新 ------------------------ */
  joystick_mutex.lock();
  if(joystick_is_connected_->data == true){
    ui_->light_joystick->setText("A");
  }else{
    ui_->light_joystick->setText("-");
  }
  joystick_mutex.unlock();

  /* ----------------------- Depth Img & Reflective Imgの更新 ------------------------ */ //途中
  points_processor_mutex.lock();
  auto depth_ros2_img = depth_img_;
  auto ref_ros2_img = ref_img_;
  depth_img_ = nullptr;
  ref_img_ = nullptr;
  points_processor_mutex.unlock();
  if(depth_ros2_img.get()){
    cv::Mat depth_img = cv_bridge::toCvShare(depth_ros2_img, depth_ros2_img->encoding)->image;
    cv::Mat depth_label_img = cv::Mat(depth_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    /*** 近いほどHが0に近く、遠いほどHが120に近くなるようにする。 depth_ptr[x]にそのままの距離の値が入っているのか違うのかはわからない***/
    for(int y = 0, rows = depth_label_img.rows; y < rows; y++){
      auto *depth_ptr = depth_img.ptr<double>(y);
      auto *depth_label_ptr = depth_label_img.ptr<cv::Vec3b>(y);
      for(int x = 0, cols = depth_label_img.cols; x < cols; x++){
        /** そのままの距離の値が入っていると仮定する **///yamlにパラメータとして入れてもいいかも
        if(depth_ptr[x] > 0 && depth_ptr[x] <= depth_max_) depth_label_ptr[x] = cv::Vec3b((120 / depth_max_) * depth_ptr[x], 255, 255);
      }
    }
    cv::cvtColor(depth_label_img, depth_label_img, cv::COLOR_HSV2BGR);
    monitor2023::drawQtImg(depth_label_img, ui_->img_depth);
  }

  if(ref_ros2_img.get()){
    cv::Mat ref_img = cv_bridge::toCvShare(ref_ros2_img, ref_ros2_img->encoding)->image;
    cv::Mat ref_label_img = cv::Mat(ref_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    /*** 強いほどHが0に近く、弱いほどHが120に近くなるようにする。おそらくref_ptr[x]は0〜1の範囲にある ***/
    for(int y = 0, rows = ref_label_img.rows; y < rows; y++){
      auto *ref_ptr = ref_img.ptr<double>(y);
      auto *ref_label_ptr = ref_label_img.ptr<cv::Vec3b>(y);
      for(int x = 0, cols = ref_label_img.cols; x < cols; x++){
        if(ref_ptr[x] != 0) ref_label_ptr[x] = cv::Vec3b(120 - 120 * ref_ptr[x], 255, 255);
      }
    }
    cv::cvtColor(ref_label_img, ref_label_img, cv::COLOR_HSV2BGR);
    monitor2023::drawQtImg(ref_label_img, ui_->img_reflectivity);
  }

  /* ----------------------- Locatorの更新 ------------------------ */
  location_mutex.lock();
  auto scan_ros_img = location_scanmatting_img_;
  auto location_pose = location_pose_;
  location_scanmatting_img_ = nullptr;
  location_pose_ = nullptr;
  location_mutex.unlock();
  // if(location_pose.get()) map_rotate_trim_.setLocation(location_pose);
  if(scan_ros_img.get()){
    cv::Mat scan_img = cv_bridge::toCvShare(scan_ros_img, scan_ros_img->encoding)->image;
    monitor2023::drawQtImg(scan_img, ui_->img_matcing_img);
  }


  /* ----------------------- WayPointの更新 ------------------------ */
  waypoint_mutex.lock();
  if(waypoint_manager_is_active_->data == true){
    ui_->light_waypoint->setText("A");
  }else{
    ui_->light_waypoint->setText("-");
  }
  auto waypoint_ros_img = waypoint_img_;
  waypoint_img_ = nullptr;
  // auto waypoint = waypoint_;
  auto waypoint_file = waypoint_file_;
  auto waypoint_index = waypoint_index_num_;
  waypoint_mutex.unlock();
  if(waypoint_ros_img.get()){
    cv::Mat waypoint_img = cv_bridge::toCvShare(waypoint_ros_img, waypoint_ros_img->encoding)->image;
    monitor2023::drawQtImg(waypoint_img, ui_->img_map);
  }
  if(waypoint_file.get()){
    ui_->box_waypoint_file->setText(waypoint_file->data.c_str());
  }
  if(waypoint_index.get()){
    ui_->box_waypoint_index->setText(std::to_string(waypoint_index->data).c_str());
  }
  // if(waypoint.get()){
  //   if(waypoint_pre_->position.x != waypoint->position.x || waypoint_pre_->position.y != waypoint->position.y || waypoint_pre_->mode != waypoint->mode){
  //     map_rotate_trim_.setWaypoint(waypoint);
  //     waypoint_pre_ = waypoint;
  //   }
  //   if(waypoint_file.get()){
  //     /* waypointファイルが更新された時 */
  //     if(waypoint_file_pre_ != waypoint_file->data){
  //       map_rotate_trim_.setWaypoint(waypoint_file->data);
  //       waypoint_file_pre_ = waypoint_file->data;
  //     }
  //   }
  // }

  /* ----------------------- Obstacle Detectorの更新 ------------------------ */
  obstacle_mutex.lock();
  auto obstacle_ros_img = obstacle_img_;
  obstacle_img_ = nullptr;
  obstacle_mutex.unlock();
  if(obstacle_ros_img.get()){
    cv::Mat obstacle_img = cv_bridge::toCvShare(obstacle_ros_img, obstacle_ros_img->encoding)->image;
    monitor2023::drawQtImg(obstacle_img, ui_->img_obstacle_detector);
  }

  /* ----------------------- Path Planningの更新 ------------------------ */
  path_planning_mutex.lock();
  if(path_planning_is_active_->data){
    ui_->light_path_planning->setText(std::to_string(path_planning_task_num_->data).c_str());
  }else{
    ui_->light_path_planning->setText("-");
  } 
  auto target_odom = path_planning_target_odmetry_;
  path_planning_mutex.unlock();
  if(target_odom.get()){
    // map_rotate_trim_.setTarget(target_odom);
    std::snprintf(str, sizeof(str), "%.2lf", target_odom->twist.twist.linear.x);
    ui_->label_potential_x->setText(str);
    std::snprintf(str, sizeof(str), "%.2lf", target_odom->twist.twist.linear.y);
    ui_->label_potential_yaw->setText(str);
  }

  /* ----------------------- PathFollowingの更新 ------------------------ */
  path_following_mutex.lock();
  if(path_following_is_active_->data == true){
    ui_->light_path_following->setText("A");
  }else{
    ui_->light_path_following->setText("-");
  }
  path_following_mutex.unlock();

  /* ----------------------- Loggerの更新 ------------------------ */
  logger_mutex.lock();
  if(logger_is_active_->data == true){
    ui_->light_logger->setText("A");
  }else{
    ui_->light_logger->setText("-");
  }
  logger_mutex.unlock();

  /* ----------------------- Robot Controlのvelocityの更新 ------------------------ */
  cmd_robot_vel_mutex.lock();
  auto cmd_robot_vel = cmd_robot_vel_;
  cmd_robot_vel_ = nullptr;
  cmd_robot_vel_mutex.unlock();
  if(cmd_robot_vel.get()){
    std::snprintf(str, sizeof(str), "%.2f", cmd_robot_vel->linear.x);
    ui_->label_velocity_x->setText(str);
    std::snprintf(str, sizeof(str), "%.2f", cmd_robot_vel->angular.x);
    ui_->label_velocity_yaw->setText(str);
  }

  /* ----------------------- Mercury Status(自律、Manual等)の表示 ------------------------ */
  if((ui_->light_Mercury_run->text() == "A") && (ui_->light_path_following->text() == "-")){
    ui_->Mercury->setStyleSheet("background-color:rgb(52, 101, 164); color:rgb(255, 255, 255);"); //青(Manual)
  }else if((ui_->light_Mercury_run->text() == "A") && (ui_->light_path_following->text() == "A")){
    ui_->Mercury->setStyleSheet("background-color:orange; color:rgb(255, 255, 255);"); //オレンジ(自律)
  }else{
    ui_->Mercury->setStyleSheet("background-color:red; color:rgb(255, 255, 255);"); //赤(制御不能？)
  }

  /* ----------------------- FrontCameraの更新(Road sign, traffic light, containerの情報とかも表示できるよ) ------------------------ */
  camera_mutex.lock();
  auto camera_ros_img = camera_img_;
  camera_img_ = nullptr;
  camera_mutex.unlock();
  if(camera_ros_img.get()){
    cv::Mat camera_img = cv_bridge::toCvShare(camera_ros_img, camera_ros_img->encoding)->image;
    monitor2023::drawQtImg(camera_img, ui_->img_camera);
  }

  /* ----------------------- Milageの更新 ------------------------ */
  mileage_mutex.lock();
  auto mileage = mileage_;
  mileage_mutex.unlock();
  if(mileage.get()) ui_->box_mileage->setText(std::to_string(mileage->data).c_str());

  /* ----------------------- LogInfoの更新 ------------------------ */
  loginfo_mutex.lock();
  auto log_filename = log_filename_;
  auto log_auto = log_auto_;
  auto log_manual = log_manual_;
  loginfo_mutex.unlock();
  if(log_filename.get()) ui_->box_filename->setText(log_filename->data.c_str());
  if(log_auto.get()) ui_->box_auto->setText(std::to_string(log_auto->data).c_str());
  if(log_manual.get()) ui_->box_manual->setText(std::to_string(log_manual->data).c_str());

  /* ----------------------- RoadSignの更新 ------------------------ */
  /* ----------------------- Containerの更新 ------------------------ */
  /* ----------------------- Traffic lightの更新 ------------------------ */

  /*** マップ表示 ***/ //map_rotate_trim_.getTrimImg()がMatでない可能性大
  // if(map_rotate_trim_.getUpdateState()) monitor2023::drawQtImg(map_rotate_trim_.getTrimImg(), ui_->img_map);

}
