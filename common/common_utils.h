#ifndef _UTILS_COMMON_UTILS_H_
#define _UTILS_COMMON_UTILS_H_

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <set>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <cstring>
#include <Eigen/Dense>
#include <chrono> 
#include <vector>
#include <cnpy.h>
#include <random>
#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>

namespace kamaz {
  namespace hagen{
class CommonUtils {
 public:
  
  CommonUtils() = default;
  ~CommonUtils() = default;
  void generate_samples_from_ellipsoid(Eigen::MatrixXf covmat
            , Eigen::Matrix3f rotation_mat, Eigen::VectorXf cent
            , Eigen::MatrixXf& container);
  
  void get_roration_matrix(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Matrix3f& r);
  
};

}  
}
#endif  // _UTILS_COMMON_UTILS_H_
