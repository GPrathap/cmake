#ifndef RRT_TREE_RRTSTAR_H
#define RRT_TREE_RRTSTAR_H

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
#include <limits>
#include <algorithm>
#include <functional>
#include <array>

#include "rrt.h"

namespace kamaz {
namespace hagen {
        class RRTStar: public RRT{
            public:
                RRTStar(SearchSpace search_space, std::vector<Eigen::VectorXd> lengths_of_edges
                , Eigen::VectorXd start_pose, Eigen::VectorXd goal_pose, int _max_samples
                , int resolution, float pro, int _rewrite_count);
                ~RRTStar() = default;



                std::vector<std::tuple<float, Eigen::VectorXd>> get_nearby_vertices(int tree, Eigen::VectorXd x_init, Eigen::VectorXd x_new);
                int current_rewrite_count(int tree);
                void rewrite(int tree, Eigen::VectorXd x_new, std::vector<std::tuple<float, Eigen::VectorXd>> L_near);
                void connect_shortest_valid(int tree, Eigen::VectorXd x_new, std::vector<std::tuple<float, Eigen::VectorXd>> L_near);
                std::vector<Eigen::VectorXd> rrt_star();

 
                int rewrite_count;
                float c_best;
                
        };
    }
}
#endif