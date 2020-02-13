#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H


#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
// #include "grad_spline/sdf_map.h"
#include <boost/functional/hash.hpp>
#include "rrt_star_3d.h"
#include "../common/search_space.h"
#include "../utils/common_utils.h"
#include <queue>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/move/move.hpp>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <numeric>   
#include <algorithm> 


namespace asio = boost::asio; 
using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace kamaz {
namespace hagen {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30



template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const& matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};


typedef Eigen::Spline<double, 3> Spline3d;
class KinodynamicRRTstar
{
private:

  
 

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);

  void push_job(kamaz::hagen::RRTStar3D* worker);

  typedef boost::packaged_task<std::vector<kamaz::hagen::PathNode>> task_t;
  typedef boost::shared_ptr<task_t> ptask_t;
  std::vector<boost::shared_future<std::vector<kamaz::hagen::PathNode>>> pending_data;
  boost::asio::io_service io_service;
  boost::thread_group threads;
  std::unique_ptr<boost::asio::io_service::work> service_work;

public:
  KinodynamicRRTstar(){};
  ~KinodynamicRRTstar();

  enum
  {
    REACH_HORIZON = 1,
    REACH_END = 2,
    NO_PATH = 3
  };

  /* main API */
  
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, int number_of_obs, bool init, std::vector<Eigen::Vector3d> curr_range, bool dynamic = false,
             double time_start = -1.0, double increase_cleareance = 0.0, int path_index = 0);

  std::vector<Eigen::Vector3d> getRRTTraj(double delta_t
                                          , std::vector<kamaz::hagen::PathNode> smoothed_path);
  double get_distance(std::vector<kamaz::hagen::PathNode> trajectory_);
  std::vector<std::vector<Eigen::Vector3d>> getRRTTrajS(double delta_t);
  Eigen::MatrixXd getSamplesRRT(double& ts, int& K);
  Eigen::MatrixXd getSamplesRRTAlternative(double& ts, int& K, bool& is_exist);
  std::vector<std::vector<kamaz::hagen::PathNode>> smoothed_paths;

  template <typename T> std::vector<size_t> sort_indexes(const std::vector<T> &v) {
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values 
    std::stable_sort(idx.begin(), idx.end(),[&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    return idx;
  }

  std::vector<double> paths_costs;
  std::vector<size_t> path_cost_indices;
  int index_of_loweres_cost = -1;
  int index_of_alternative_cost = -1;
  typedef shared_ptr<KinodynamicRRTstar> Ptr;
  bool is_using_whole_space = false;
  double rrt_star_steer_min = 4;
  double  rrt_star_steer_max = 6;
  int lqr_num_of_iteration = 20;
  double obstacle_radios = 0.4;
  bool consider_obs = true;
  int number_of_closest_obs = 10;
   /* ---------- main data structure ---------- */
  // vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  // NodeHashTable expanded_nodes_;
  // std::vector<PathNodePtr> path_nodes_;
  std::vector<kamaz::hagen::PathNode> path_rrt_;
  
  kamaz::hagen::CommonUtils common_utils;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  // shared_ptr<SDFMap> sdf_map;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_ = 0.25;
  double init_max_tau_ = 0.8;
  double max_vel_ = 3.0;
  double max_acc_ = 3.0;
  double w_time_ = 10.0;
  double horizon_;
  double lambda_heu_;
  double margin_;
  int allocate_num_;
  int check_num_;
  double tie_breaker_ = 1.0 + 1.0 / 10000;
  int number_of_paths = 4;
  int r = 1;
  int max_samples = 1000;
  int rewrite_count = 32;
  double proc = 0.1;
  int save_data_index = 0;
  double rrt_avoidance_dist = 0.6;
  double lqr_min_dis = 1.0;
  double lqr_min_dt = 0.1;
  double lqr_feasibility_max_vel = 0.25;
  double space_min_z = 1.0;
  int order_of_search_space = 4;
  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;
};

}  
}
#endif