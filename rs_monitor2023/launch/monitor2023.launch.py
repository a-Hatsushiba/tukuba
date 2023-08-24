from inspect import Parameter
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prefix = get_package_share_directory('rs_monitor2023')
    components = LoadComposableNodes(
      target_container="rs_container",
      composable_node_descriptions=[
        ComposableNode(
          package='rs_monitor2023',
          plugin='monitor2023::ROS2WrapperMonitor2023',
          name='rs_monitor2023',
          parameters=[
            join(pkg_prefix, "cfg/monitor2023.yaml")
          ],
          remappings=[
            #subscriber
            ('/lidar/top_lidar_active', '/rs_points_processor/top_lidar_active'),
            ('/lidar/middle_lidar_active', '/rs_points_processor/middle_lidar_active'),
            ('/lidar/bottom_lidar_active', '/rs_points_processor/bottom_lidar_active'),
            ('/camera/image', '/camera1/image'),
            ('/sonar/ranges', '/mercury_sonar/ranges'),
            ('/mercury/state', '/mercury/state'),
            ('/mercury_run/is_active', '/rs_waypoint_manager/is_active'),
            ('/waypoint_manager/waypoint', '/rs_waypoint_manager/waypoint'),
            ('/waypoint_manager/is_active', '/rs_waypoint_manager/is_active'),
            ('/waypoint_manager/waypoint_file', '/rs_waypoint_manager/waypoint_file'),
            ('/logger/is_active', '/rs_logger/is_active'),
            ('/path_planning/target_odom', '/rs_path_planning/target_odom'),
            ('/path_planning/task_num', '/rs_path_planning/task_num'),
            ('/path_planning/is_active', '/rs_path_planning/is_active'),
            ('/path_following/is_active', '/rs_path_following/is_active'),
            ('/locator/corrected_pose', '/locator/corrected_pose'),
            ('/locator/scanmatch_img', '/locator/scanmatch_img'),
            ('/obstacle_detector/image', '/rs_obstacle_detector/obstacle_image'),
            ('/points_processor/depth_img', '/rs_points_processor/depth_img'),
            ('/points_processor/ref_img', '/rs_points_processor/ref_img'),
            ('/joy/is_connected', '/joystick/is_connected'),
            ('/mercury/cmd_robot_vel', '/mercury/cmd_robot_vel'),
            ('/rs_logger/auto_save_cnt', '/rs_logger/auto_save_cnt'),
            ('/rs_logger/auto_save_cnt', '/rs_logger/auto_save_cnt');
          ],
          extra_arguments=[
            {'use_intra_process_comms': True}
          ]
        )
      ]
    )
    return LaunchDescription([
        components
    ])