#ifndef PATH_PLANNER_RRT_THREE_UTILS_H_
#define PATH_PLANNER_RRT_THREE_UTILS_H_

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
#include "../common/search_space.h"
#include "../common/common_utils.h"
#include "tree.h"
#include <unordered_map>
#include <stddef.h>

namespace kamaz {
namespace hagen {
        class RRTBase {
            public:
                RRTBase(SearchSpace search_space, std::vector<Eigen::VectorXf> lengths_of_edges
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose, Eigen::VectorXf first_object_found_pose, int _max_samples
                , int resolution, float pro);
                ~RRTBase() = default; 

            SearchSpace X;
            CommonUtils common_utils;
            int sample_taken;
            int max_samples;
            std::vector<Eigen::VectorXf> Q;
            int r;
            float prc;
            Eigen::VectorXf x_init;
            Eigen::VectorXf x_goal;
            Eigen::VectorXf first_detected_object_found_pose;
            std::map<int, Tree> trees;
            float min_acceptable = 0.01;
            void add_tree();
            void add_vertex(int tree, Eigen::VectorXf v);
            void add_edge(int tree, Eigen::VectorXf child, Eigen::VectorXf parent);
            std::vector<Eigen::VectorXf> nearby(int tree, Eigen::VectorXf x
                            , int max_neighbors);
            Eigen::VectorXf get_nearest(int tree, Eigen::VectorXf x);
            Eigen::VectorXf steer(Eigen::VectorXf start, Eigen::VectorXf goal
                                , float distance);
            std::vector<Eigen::VectorXf> new_and_near(int tree, Eigen::VectorXf q);
            bool connect_to_point(int tree, Eigen::VectorXf x_a, Eigen::VectorXf x_b);
            bool can_connect_to_goal(int tree);
            bool isEdge(Eigen::VectorXf point, int tree);
            Eigen::VectorXf getEdge(Eigen::VectorXf point, int tree);
            std::vector<Eigen::VectorXf> reconstruct_path(int tree
                                    , Eigen::VectorXf x_init, Eigen::VectorXf x_goal);
            bool check_none_vector(Eigen::VectorXf  vector);
            void setEdge(Eigen::VectorXf key, Eigen::VectorXf value, int tree);
            void connect_to_the_goal(int tree);
            std::vector<Eigen::VectorXf> get_path();
            bool check_solution(std::vector<Eigen::VectorXf>& path);
            int sizeOfEdge(int tree);
            void printEdge(int tree);
            bool is_equal_vectors(Eigen::VectorXf a, Eigen::VectorXf b);
            float cost_to_go(Eigen::VectorXf a, Eigen::VectorXf b);
            float path_cost(Eigen::VectorXf a, Eigen::VectorXf b, int tree);
            float segment_cost(Eigen::VectorXf a, Eigen::VectorXf b);

        };
    }
}
#endif