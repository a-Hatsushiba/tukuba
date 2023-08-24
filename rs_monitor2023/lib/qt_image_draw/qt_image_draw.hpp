#ifndef QT_IMAGE_DRAW_HPP
#define QT_IMAGE_DRAW_HPP

// #include "monitor2023.h"
#include <QLabel>
#include <opencv2/opencv.hpp>

namespace monitor2023
{

void drawQtImg(cv::Mat &img, QLabel* &label);

}
#endif // QT_IMAGE_DRAW_HPP