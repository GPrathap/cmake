#ifndef _UTILS_COMMON_UTILS_H_
#define _UTILS_COMMON_UTILS_H_

#include <stack>
#include <vector>
#include <array>
#include <iostream>
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
#include <algorithm>
#include <list>
#include <numeric>
#include <unordered_map>
#include <memory>
#include <type_traits>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <deque>
#include <atomic>

#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include "../include/colours.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


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
  
  void get_point_on_the_trajectory(Eigen::VectorXf way_point
      , Eigen::VectorXf start_point,  Eigen::VectorXf& path_position);
  float voxel_side_length;
  Eigen::Vector3f init_min_point;
  std::string world_frame_id;
};

}  
}
#endif  // _UTILS_COMMON_UTILS_H_
