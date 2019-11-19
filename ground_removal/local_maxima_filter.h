#ifndef GROUND_REMOVAL_LOCAL_MAXIMA_FILTER_H_
#define GROUND_REMOVAL_LOCAL_MAXIMA_FILTER_H_

#include <opencv2/opencv.hpp>
// #include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <cassert>
#include <tuple>
#include <set>
#include "union_find.h"
// #include <parallel/algorithm>
#include <cnpy.h>

namespace kamaz {
    namespace hagen{

    struct groups {
        double p1;
        double p2;
        Pixel p3;
    };

class LocalMaximaFilter{
 public:
    LocalMaximaFilter() = default;
    ~LocalMaximaFilter() = default;

    double get_pixel_value(Pixel p);
    void iter_neighbors(Pixel p, std::vector<Pixel> &item_list); 
    void persistence(const cv::Mat& img, cv::Mat& filtered_image, std::map<int, int>& detected_indics);
    void persistence_and_save_data(const cv::Mat& img, cv::Mat& filtered_image, int index);

    double distance(int x, int y, int i, int j);
    void print_tuple(std::tuple<int, int> object);
    double gaussian(double x, double sigma);
    void applyBilateralFilter(cv::Mat& filteredImage, int source_index, int diameter, double sigmaI, double sigmaS);

private:
    UnionFindDS uf; 
    int img_width;
    int img_height;

    std::vector<double> depth_img_row_vector;
};
}
}
#endif 