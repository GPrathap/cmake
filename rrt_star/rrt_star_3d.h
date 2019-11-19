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

               std::vector<Eigen::Vector3d> rrt_planner(SearchSpace search_space
                , Eigen::Vector3d start_pose, Eigen::Vector3d goal_pose
                , Eigen::Vector3d start_position, double obstacle_fail_safe_distance, double min_angle, CommonUtils& common_utils
                , std::atomic_bool &is_allowed_to_run);
               
               std::vector<Eigen::Vector3d> rrt_planner_and_save(SearchSpace search_space
                , Eigen::Vector3d start_pose, Eigen::Vector3d goal_pose
                , Eigen::Vector3d start_position, double obstacle_fail_safe_distance, double min_angle, CommonUtils& common_utils
                , std::atomic_bool &is_allowed_to_run, int index);

               void rrt_init(std::vector<Eigen::Vector2d> _lengths_of_edges
                    , int max_samples, int _resolution, double _pro
                    , int rewrite_count);
                    
               Eigen::Vector3d get_search_space_dim(Eigen::Vector3d dim);
               std::vector<SearchSpace::Rect> get_obstacles();
               std::vector<SearchSpace::Rect> get_random_obstacles(int number_of_obstacles
               , Eigen::VectorXd x_dimentions, Eigen::Vector3d x_init, Eigen::Vector3d x_goal);
               void save_edges(std::map<int, Tree> trees, std::string file_name);
               void save_obstacle(std::vector<SearchSpace::Rect> obstacles, std::string file_name);
               void save_poses(Eigen::Vector3d start, Eigen::Vector3d end, std::string file_name);
               void save_path(std::vector<Eigen::Vector3d> path, std::string file_name);
               void save_trajectory(std::vector<Eigen::Vector3d> trajectory_of_drone);
               double get_distance(std::vector<Eigen::Vector3d> trajectory_);


            private:
               std::vector<Eigen::Vector3d> lengths_of_edges;
               int _max_samples;
               int resolution; 
               double pro;
               int _rewrite_count;
               RRTPlannerOptions rrt_planner_options;
               std::string stotage_location;
        };
    }
}
#endif