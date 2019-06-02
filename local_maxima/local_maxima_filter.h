#ifndef SRC_LOCAL_MAXIMA_FILTER_H_
#define SRC_LOCAL_MAXIMA_FILTER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <cassert>
#include <tuple>
#include <set>
#include "union_find.h"
#include <parallel/algorithm>
#include <cnpy.h>

namespace kamaz {
    namespace hagen{
struct groups {
    float p1;
    float p2;
    std::tuple<int, int, float> p3;
};

class LocalMaximaFilter{
 public:
    LocalMaximaFilter();
    ~LocalMaximaFilter();

    float get_pixel_value(const cv::Mat& img, std::tuple<int, int, float> p);
    //float get_pixel_value(const cv::cuda::GpuMat& img, std::tuple<int, int, float> p);
    void iter_neighbors(std::tuple<int, int, float> p, int w, int h, std::vector<std::tuple<int, int, float>> &item_list, const cv::Mat& img); 
    void persistence(const cv::Mat& img, std::vector<std::tuple<std::tuple<int, int, float>, groups>>& result);
    void persistence(const cv::Mat& img, cv::Mat& filtered_image);
    void persistence_and_save_data(const cv::Mat& img, cv::Mat& filtered_image, int index);

    float distance(int x, int y, int i, int j);
    void print_tuple(std::tuple<int, int> object);
    float gaussian(float x, float sigma);
    void applyBilateralFilter(const cv::Mat& source, cv::Mat& filteredImage, int x, int y, int diameter, int h, int w, float sigmaI, float sigmaS);

private:
    UnionFindDS* uf;
    
};
}
}
#endif 