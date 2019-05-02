#ifndef RRT_TREE_STAR3D_H
#define RRT_TREE_STAR3D_H

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
#include<cstdlib>
#include<iostream>
#include<map>
#include<string>

#include "rrtstar.h"

namespace kamaz {
namespace hagen {
        class RRTStar3D {
            public:
               RRTStar3D() = default;
               ~RRTStar3D() = default;

               std::vector<Eigen::VectorXd> rrt_planner(SearchSpace search_space
                , Eigen::VectorXd start_pose, Eigen::VectorXd goal_pose); 
               
               std::vector<Eigen::VectorXd> rrt_planner_and_save(SearchSpace search_space
                , Eigen::VectorXd start_pose, Eigen::VectorXd goal_pose);
               void rrt_init(std::vector<Eigen::VectorXd> _lengths_of_edges
                    , int max_samples, int _resolution, float _pro
                    , int rewrite_count);
               Eigen::VectorXd get_search_space_dim(Eigen::VectorXd dim);
               std::vector<SearchSpace::Rect> get_obstacles();
               std::vector<SearchSpace::Rect> get_random_obstacles(int number_of_obstacles, Eigen::VectorXd x_dimentions);
               void save_edges(std::map<int, Tree> trees);
               void save_obstacle(std::vector<SearchSpace::Rect> obstacles);
               void save_poses(Eigen::VectorXd start, Eigen::VectorXd end);
               void save_path(std::vector<Eigen::VectorXd> path);

            private:
               std::vector<Eigen::VectorXd> lengths_of_edges;
               int _max_samples;
               int resolution; 
               float pro;
               int _rewrite_count;
        };
    }
}
#endif