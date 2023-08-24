#include <opencv2/opencv.hpp>
#include "map_make.hpp"

namespace monitor2023
{

void MainMapCreate::setMapImg(cv::Mat img){
  map_img_ = img;
}

void MainMapCreate::setGridSize(const double size){
  
}

void MainMapCreate::setMapViewSize(const double size){

}

void MainMapCreate::setLocation(const Pose pose){

}

void MainMapCreate::setWaypoint(const Waypoint waypoint){

}

void MainMapCreate::setWaypoints(){

}

void setTarget(const Odometry odom){

}

bool getUpdateState(){

}

cv::Mat getTrimImg(){

}

}