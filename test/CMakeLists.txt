cmake_minimum_required(VERSION 3.0)
project(test VERSION 1.1.0 LANGUAGES CXX)
# ビルドフラグの設定
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
# インクルードパスの追加
if(WIN32)
include_directories(C:/local/include)
link_directories(C:/local/lib)
else()
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
endif()
# Find Package
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) #インクルードディレクトリ指定
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${HEADER_DIR}
)
# ビルドに関する設定
set(EXECUTABLES
    # "rectangle_detection_vol1";
    # "rectangle_detection_vol2";
    # "container_color_extract";
    # "rate_calculation";
    # "hsv_trackbar";
    "character_recognition";
    "image_editing";
    "camera_container";
    "lidar_container";
    # "map_make";
    "waypoint_tool";
    # "generate_waypoint";
    # "rikuto_waypoint_tool"
    )
set(DEPENDENT_LIBS rs_common;rs_navi)
# set(DEPENDENT_LIBS opencv_core;opencv_highgui;opencv_videoio;opencv_imgproc;opencv_imgcodecs;rs_common;rs_navi)
foreach(target IN LISTS EXECUTABLES)
add_executable(${target} src/${target}.cpp)
target_link_libraries(${target} ${DEPENDENT_LIBS})
target_link_libraries(${target} ${OpenCV_LIBS})
endforeach()

# blue_container用
add_executable(blue_container src/blue_container.cpp)
add_library(CAMERA SHARED src/camera_container.cpp include/camera_container.hpp)
add_library(LiDAR SHARED src/lidar_container.cpp include/lidar_container.hpp)
target_link_libraries(blue_container CAMERA LiDAR)
target_link_libraries(blue_container ${DEPENDENT_LIBS})
target_link_libraries(blue_container ${OpenCV_LIBS})

