cmake_minimum_required(VERSION 3.5)
project(rs_monitor2023)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)
set(ROS_LIBS rclcpp;rclcpp_components;std_msgs;geometry_msgs;sensor_msgs)
set(ROS_LIBS ${ROS_LIBS};nav_msgs;cv_bridge;tf2)
set(ROS_LIBS ${ROS_LIBS};OpenCV;Qt5Widgets;Qt5Core;Qt5Gui)
set(ROS_LIBS ${ROS_LIBS};ros_robocore_interfaces;ros2_rs_miyauchi_interfaces)
foreach(lib IN LISTS ROS_LIBS)
  find_package(${lib} REQUIRED)
endforeach()

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};$ENV{LIB}/qt5/$ENV{QT_DISTRO})
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

include_directories(
  include
  lib
  ../ProjectRyusei.waypoint/include
  ../ProjectRyusei.miyauchi/ProjectRyusei.waypoint/include
  ../ProjectRyusei.miyauchi/ProjectRyusei.waypoint/lib
)

add_library(${PROJECT_NAME} SHARED
  src/monitor2023_wrapper.cpp
  src/monitor2023.cpp
  src/monitor2023.ui
  lib/qt_image_draw/qt_image_draw.cpp
  lib/qt_image_draw/qt_image_draw.hpp
  # lib/map_make/map_make.cpp
  # lib/map_make/map_make.hpp
  include/rs_monitor2023/monitor2023.h
  include/rs_monitor2023/ui_monitor2023.h
)

ament_target_dependencies(${PROJECT_NAME} ${ROS_LIBS})
rclcpp_components_register_nodes(${PROJECT_NAME} "monitor2023::ROS2WrapperMonitor2023")
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

ament_package()
