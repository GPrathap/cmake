#ifndef PATH_PLANNER_RRT_H_
#define PATH_PLANNER_RRT_H_

#include "rrtbase.h"

namespace kamaz {
namespace hagen {
        class RRT : public RRTBase {
            public:
                RRT(SearchSpace search_space, std::vector<Eigen::VectorXf> lengths_of_edges
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose, Eigen::VectorXf first_object_found_pose, int _max_samples
                , int resolution, float pro);
                ~RRT() = default;
                std::vector<Eigen::VectorXf> rrt_search();
        };
    }
}
#endif