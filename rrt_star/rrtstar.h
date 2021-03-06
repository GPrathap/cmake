#ifndef PATH_PLANNER_RRT_TREE_RRTSTAR_H_
#define PATH_PLANNER_RRT_TREE_RRTSTAR_H_

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
                RRTStar(RRTPlannerOptions options,  int rewrite_count, CommonUtils& common_utils, std::atomic_bool &is_allowed_to_run);
                ~RRTStar() = default;

                std::vector<std::tuple<float, Eigen::VectorXf>> get_nearby_vertices(int tree, Eigen::VectorXf x_init, Eigen::VectorXf x_new);
                int current_rewrite_count(int tree);
                void rewrite(int tree, Eigen::VectorXf x_new, std::vector<std::tuple<float, Eigen::VectorXf>> L_near);
                void connect_shortest_valid(int tree, Eigen::VectorXf x_new, std::vector<std::tuple<float, Eigen::VectorXf>> L_near);
                std::vector<Eigen::VectorXf> rrt_star();
                int rewrite_count;
                float c_best;
                
        };
    }
}
#endif