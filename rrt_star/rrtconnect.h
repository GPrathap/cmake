#ifndef RRT_TREE_CONNECT_TREE_H
#define RRT_TREE_CONNECT_TREE_H

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
                RRTConnect(SearchSpace search_space, std::vector<Eigen::VectorXd> lengths_of_edges
                , Eigen::VectorXd start_pose, Eigen::VectorXd goal_pose, int _max_samples
                , int resolution, float pro);
                ~RRTConnect() = default;
                
                bool swapped;
                void swap_trees();
                void unswap();
                RRTStatus extend(int tree, Eigen::VectorXd x_rand, Eigen::VectorXd& x_final);
                RRTStatus connect(int tree, Eigen::VectorXd x, Eigen::VectorXd x_connect);
                std::vector<Eigen::VectorXd> rrt_connect();
        };
    }
}
#endif