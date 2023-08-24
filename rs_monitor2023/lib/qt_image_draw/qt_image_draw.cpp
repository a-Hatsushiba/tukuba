#include "qt_image_draw.hpp"

namespace monitor2023
{

void drawQtImg(cv::Mat &img, QLabel* &label)
{
  cv::Mat rgb, conv;
  int depth = img.depth();
  // if(depth != CV_8U){
  //   img.convertTo(conv, CV_8U);
  //   img = conv.clone();
  // }
  cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
  int w = label->width();
  int h = label->height();
  label->setPixmap(QPixmap::fromImage(
                    QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).scaled(w, h, Qt::KeepAspectRatio)));
}

}