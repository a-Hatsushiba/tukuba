#ifndef MONITOR2023_H
#define MONITOR2023_H

#include <opencv2/opencv.hpp>
#include <mutex>

/** ROS2 **/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
//独自の型
#include "ros2_rs_miyauchi_interfaces/msg/way_point.hpp"
//revast msg
#include "ros_robocore_interfaces/msg/robot_state_msg.hpp"

/** Qt **/
#include <QTimer>
#include <QMainWindow>

using namespace std;
using std_msgs::msg::Bool;
using std_msgs::msg::Empty;
using sensor_msgs::msg::Image;
using std_msgs::msg::Float32MultiArray;
using ros_robocore_interfaces::msg::RobotStateMsg;
using std_msgs::msg::String;
using ros2_rs_miyauchi_interfaces::msg::WayPoint;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Int32;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
// namespace rs = project_ryusei;


QT_BEGIN_NAMESPACE
namespace Ui { class Monitor2023; }
QT_END_NAMESPACE

class Monitor2023 : public QMainWindow
{
    Q_OBJECT

public:
    Monitor2023(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~Monitor2023();

private:
    /** 3DliDAR(H)関連 **/
    bool lidar_h_is_received = false;
    mutex lidar_h_mutex;
    rclcpp::Subscription<Empty>::SharedPtr sub_lidar_h_;
    void onHighLidarSubscribed(const Empty::SharedPtr empty);

    /** 3DliDAR(M)関連 **/
    bool lidar_m_is_received = false;
    mutex lidar_m_mutex;
    rclcpp::Subscription<Empty>::SharedPtr sub_lidar_m_;
    void onMidLidarSubscribed(const Empty::SharedPtr empty);

    /** 2DliDAR(L)関連 **/
    bool lidar_l_is_received = false;
    mutex lidar_l_mutex;
    rclcpp::Subscription<Empty>::SharedPtr sub_lidar_l_;
    void onLowLidarSubscribed(const Empty::SharedPtr empty);

    /** カメラ関連 **/
    bool camera_is_received = false;
    mutex camera_mutex;
    Image::SharedPtr camera_img_;
    rclcpp::Subscription<Image>::SharedPtr sub_camera_img_;
    void onCameraImgSubscribed(const Image::SharedPtr img);

    /** Sonar関連 **/
    bool sonar_range_is_received = false;
    mutex sonar_range_mutex;
    Float32MultiArray::SharedPtr sonar_range_;
    rclcpp::Subscription<Float32MultiArray>::SharedPtr sub_sonar_range_;
    void onSonarRangeSubscribed(const Float32MultiArray::SharedPtr range);

    /** Mercury関連 **/
    bool mercury_is_received = false;
    mutex mercury_mutex;
    RobotStateMsg::SharedPtr mercury_state_;
    rclcpp::Subscription<RobotStateMsg>::SharedPtr sub_mercury_state_;
    void onMercuryStateSubscribed(const RobotStateMsg::SharedPtr state);

    /** Mercury run関連 **/
    bool mercury_run_is_received = false;
    mutex mercury_run_mutex;
    Bool::SharedPtr mercury_run_is_active_;
    rclcpp::Subscription<Bool>::SharedPtr sub_mercury_run_;
    void onMercuryRunSubscribed(const Bool::SharedPtr active);

    /** Waypoint関連 **/
    bool waypoint_is_received = false;
    mutex waypoint_mutex;
    Bool::SharedPtr waypoint_manager_is_active_;
    rclcpp::Subscription<Bool>::SharedPtr sub_waypoint_manager_is_active_;
    void onWaypointManagerActiveSubscribed(const Bool::SharedPtr active);
    Image::SharedPtr waypoint_img_;
    rclcpp::Subscription<Image>::SharedPtr sub_waypoint_img_;
    void onWaypointImgSubscribed(const Image::SharedPtr img);
    String::SharedPtr waypoint_file_;
    rclcpp::Subscription<String>::SharedPtr sub_waypoint_file_;
    void onWaypointFileSubscribed(const String::SharedPtr filename);
    Int32::SharedPtr waypoint_index_num_;
    rclcpp::Subscription<Int32>::SharedPtr sub_waypoint_index_num_;
    void onWaypointIndexNumSubscribed(const Int32::SharedPtr num);
    // WayPoint::SharedPtr waypoint_;
    // rclcpp::Subscription<Waypoint>::SharedPtr sub_waypoint_;
    // void onWaypointSubscribed(const Waypoint::SharedPtr waypoint);

    /** Logger関連 **/
    bool logger_is_received = false;
    mutex logger_mutex;
    Bool::SharedPtr logger_is_active_;
    rclcpp::Subscription<Bool>::SharedPtr sub_logger_is_active_;
    void onLoggerActiveSubscribed(const Bool::SharedPtr active);

    /** PathPlanning関連 **/
    bool path_planning_is_received = false;
    mutex path_planning_mutex;
    Odometry::SharedPtr path_planning_target_odmetry_; 
    Int32::SharedPtr path_planning_task_num_;
    Bool::SharedPtr path_planning_is_active_;
    rclcpp::Subscription<Odometry>::SharedPtr sub_path_planning_target_odometry_;
    rclcpp::Subscription<Int32>::SharedPtr sub_path_planning_task_num_;
    rclcpp::Subscription<Bool>::SharedPtr sub_path_planning_is_active_;
    void onPathPlanningTargetOdometrySubscribed(const Odometry::SharedPtr odometry);
    void onPathPlanningTaskNumSubscribed(const Int32::SharedPtr num);
    void onPathPlanningActiveSubscribed(const Bool::SharedPtr active);

    /** PathFollowing関連 **/
    bool path_following_is_received = false;
    mutex path_following_mutex;
    Bool::SharedPtr path_following_is_active_;
    rclcpp::Subscription<Bool>::SharedPtr sub_path_following_is_active_;
    void onPathFollowingActiveSubscribed(const Bool::SharedPtr active);
    
    /** Locator関連 **/
    bool location_is_received = false;
    mutex location_mutex;
    Pose::SharedPtr location_pose_;
    Image::SharedPtr location_scanmatting_img_;
    rclcpp::Subscription<Pose>::SharedPtr sub_location_pose_;
    rclcpp::Subscription<Image>::SharedPtr sub_location_scanmatting_img_;
    void onLocationPoseSubscribed(const Pose::SharedPtr pose);
    void onLocationScanMattingImgSubscribed(const Image::SharedPtr img);

    /** Obstacle関連 **/
    bool obstacle_is_received = false;
    mutex obstacle_mutex;
    Image::SharedPtr obstacle_img_;
    rclcpp::Subscription<Image>::SharedPtr sub_obstacle_img_;
    void onObstacleImgSubscribed(const Image::SharedPtr img);

    /** Points Processor(距離画像、反射強度画像)関連 **/
    double depth_max_;
    bool points_processor_is_received = false;
    mutex points_processor_mutex;
    Image::SharedPtr depth_img_;
    Image::SharedPtr ref_img_;
    rclcpp::Subscription<Image>::SharedPtr sub_depth_img_;
    rclcpp::Subscription<Image>::SharedPtr sub_ref_img_;
    void onDepthImgSubscribed(const Image::SharedPtr img);
    void onRefImgSubscribed(const Image::SharedPtr img);

    /** Joystick関連 **/
    bool joystick_is_received = false;
    mutex joystick_mutex;
    Bool::SharedPtr joystick_is_connected_;
    rclcpp::Subscription<Bool>::SharedPtr sub_joustick_is_connected_;
    void onJoystickConnectSubscribed(const Bool::SharedPtr connect);

    /** mercury制御パラメータ関連 **/
    mutex cmd_robot_vel_mutex;
    Twist::SharedPtr cmd_robot_vel_;
    rclcpp::Subscription<Twist>::SharedPtr sub_cmd_robot_vel_;
    void onCmdRobotVelSubscribed(const Twist::SharedPtr vel);

    /** mileage **/
    mutex mileage_mutex;
    Int32::SharedPtr mileage_;
    rclcpp::Subscription<Int32>::SharedPtr sub_mileage_;
    void onMileageSubscribed(const Int32::SharedPtr mile);

    /** Log(ファイル名、数) **/
    mutex loginfo_mutex;
    String::SharedPtr log_filename_; //他のyamlのファイルから読み込む予定
    Int32::SharedPtr log_auto_;
    Int32::SharedPtr log_manual_;
    // rclcpp::Subscription<String>::SharedPtr sub_log_filename_;
    rclcpp::Subscription<Int32>::SharedPtr sub_log_auto_;
    rclcpp::Subscription<Int32>::SharedPtr sub_log_manual_;
    // void onLogFilenameSubscribed(const String::SharedPtr name);
    void onLogAutoSubscribed(const Int32::SharedPtr num);
    void onLogManualSubscribed(const Int32::SharedPtr num);

    /*** このあと3つは後々追記 ***/
    /** Road Sign関連 **/
    bool road_sign_is_received = false;
    mutex road_sign_mutex;

    /** Container関連 **/
    bool container_is_received = false;
    mutex container_mutex;

    /** Traffic light関連 **/
    bool traffic_light_is_received = false;
    mutex traffic_light_mutex;

    /** Map描画用 **/
    // std::string waypoint_file_pre_;
    // WayPoint::SharedPtr waypoint_pre_ = std::make_shared<WayPoint>();
    // rs::MapRotateTrim map_rotate_trim_;

    /** Monitor(Qt)用 **/
    std::unique_ptr<Ui::Monitor2023> ui_;
    rclcpp::Node::SharedPtr node_;
    void initUi();
    void initParam();
    void initSubscriber();
    void initQtimer();
    void updateNodeStatus();
    void updateGui();
};
#endif // MONITOR2023_H
