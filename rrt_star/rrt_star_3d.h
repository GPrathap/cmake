#ifndef PATH_PLANNER_RRT_TREE_STAR3D_H_
#define PATH_PLANNER_RRT_TREE_STAR3D_H_

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
#include <cnpy.h>
#include<complex>
#include<ctime>
#include<cstdlib>
#include<iostream>
#include<map>
#include<string>
#include "../utils/common_utils.h"
#include "rrtstar.h"
#include "rrtbase.h"
#include "../common/trajectory_smoother.h"

namespace kamaz {
namespace hagen {
        class RRTStar3D {
            public:
               RRTStar3D() = default;
               ~RRTStar3D() = default;

               std::vector<Eigen::VectorXf> rrt_planner(SearchSpace search_space
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose
                , Eigen::VectorXf start_position, float obstacle_fail_safe_distance, float min_angle, CommonUtils& common_utils
                , std::atomic_bool &is_allowed_to_run);
               
               std::vector<Eigen::VectorXf> rrt_planner_and_save(SearchSpace search_space
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose
                , Eigen::VectorXf start_position, float obstacle_fail_safe_distance, float min_angle, CommonUtils& common_utils
                , std::atomic_bool &is_allowed_to_run, int index);

               void rrt_init(std::vector<Eigen::VectorXf> _lengths_of_edges
                    , int max_samples, int _resolution, float _pro
                    , int rewrite_count);
                    
               Eigen::VectorXf get_search_space_dim(Eigen::VectorXf dim);
               std::vector<SearchSpace::Rect> get_obstacles();
               std::vector<SearchSpace::Rect> get_random_obstacles(int number_of_obstacles, Eigen::VectorXf x_dimentions);
               void save_edges(std::map<int, Tree> trees, std::string file_name);
               void save_obstacle(std::vector<SearchSpace::Rect> obstacles, std::string file_name);
               void save_poses(Eigen::VectorXf start, Eigen::VectorXf end, std::string file_name);
               void save_path(std::vector<Eigen::VectorXf> path, std::string file_name);
               void save_trajectory(std::vector<Eigen::VectorXf> trajectory_of_drone);
               float get_distance(std::vector<Eigen::VectorXf> trajectory_);


            private:
               std::vector<Eigen::VectorXf> lengths_of_edges;
               int _max_samples;
               int resolution; 
               float pro;
               int _rewrite_count;
               RRTPlannerOptions rrt_planner_options;
               std::string stotage_location;
        };
    }
}
#endif