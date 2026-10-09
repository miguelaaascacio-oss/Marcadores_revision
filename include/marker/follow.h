#ifndef FOLLOW_H
#define FOLLOW_H 1

#include "triangles.h"
#include "opencv2/core/mat.hpp"

int follow_perimeter( cv::Mat& Img, int, int, uchar, POINT * );

#endif
