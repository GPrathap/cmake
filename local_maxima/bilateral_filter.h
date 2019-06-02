#ifndef SRCP_BILATERAL_FILTER_H_
#define SRCP_BILATERAL_FILTER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <cassert>
#include <tuple>
#include <set>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

namespace kamaz {

class BilateralFilter{
 public:
    BilateralFilter() = default;
    ~BilateralFilter() = default;

    float distance(int x, int y, int i, int j);
    double gaussian(float x, double sigma);
    void applyBilateralFilter(cv::Mat source, cv::Mat filteredImage, int x, int y, int diameter, double sigmaI, double sigmaS);
    cv::Mat bilateralFilterOwn(cv::Mat source, int diameter, double sigmaI, double sigmaS);
};
}
#endif 