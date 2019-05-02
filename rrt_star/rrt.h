#ifndef RRT_H
#define RRT_H

#include "rrtbase.h"

namespace kamaz {
namespace hagen {
        class RRT : public RRTBase {
            public:
                RRT(SearchSpace search_space, std::vector<Eigen::VectorXd> lengths_of_edges
                , Eigen::VectorXd start_pose, Eigen::VectorXd goal_pose, int _max_samples
                , int resolution, float pro);
                ~RRT() = default;
                std::vector<Eigen::VectorXd> rrt_search();
        };
    }
}
#endif