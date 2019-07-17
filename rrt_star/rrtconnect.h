#ifndef PATH_PLANNER_RRT_TREE_CONNECT_TREE_H_
#define PATH_PLANNER_RRT_TREE_CONNECT_TREE_H_

#include "rrtbase.h"

namespace kamaz {
namespace hagen { 
        enum Status{
                    FAILED = 1,
                    TRAPPED = 2,
                    ADVANCED = 3,
                    REACHED = 4
        };
 
        typedef Status RRTStatus;
        class RRTConnect : public RRTBase {
            public:
                RRTConnect(SearchSpace search_space, std::vector<Eigen::VectorXf> lengths_of_edges
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose, Eigen::VectorXf ostacle_pose, int _max_samples
                , int resolution, float pro);
                ~RRTConnect() = default;
                
                bool swapped;
                void swap_trees();
                void unswap();
                RRTStatus extend(int tree, Eigen::VectorXf x_rand, Eigen::VectorXf& x_final);
                RRTStatus connect(int tree, Eigen::VectorXf x, Eigen::VectorXf x_connect);
                std::vector<Eigen::VectorXf> rrt_connect();
        };
    }
}
#endif