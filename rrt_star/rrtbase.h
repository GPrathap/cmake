#ifndef RRT_THREE_UTILS_H
#define RRT_THREE_UTILS_H

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
#include "search_space.h"
#include "tree.h"

namespace kamaz {
namespace hagen {
        class RRTBase {
            public:
                RRTBase(SearchSpace search_space, std::vector<Eigen::VectorXd> lengths_of_edges
                , Eigen::VectorXd start_pose, Eigen::VectorXd goal_pose, int _max_samples
                , int resolution, float pro);
                ~RRTBase() = default; 

            SearchSpace X;
            int sample_taken;
            int max_samples;
            std::vector<Eigen::VectorXd> Q;
            int r;
            float prc;
            Eigen::VectorXd x_init;
            Eigen::VectorXd x_goal;
            std::map<int, Tree> trees;
            float min_acceptable = 0.01;
            void add_tree();
            void add_vertex(int tree, Eigen::VectorXd v);
            void add_edge(int tree, Eigen::VectorXd child, Eigen::VectorXd parent);
            std::vector<Eigen::VectorXd> nearby(int tree, Eigen::VectorXd x, int max_neighbors);
            Eigen::VectorXd get_nearest(int tree, Eigen::VectorXd x);
            Eigen::VectorXd steer(Eigen::VectorXd start, Eigen::VectorXd goal, float distance);
            std::vector<Eigen::VectorXd> new_and_near(int tree, Eigen::VectorXd q);
            bool connect_to_point(int tree, Eigen::VectorXd x_a, Eigen::VectorXd x_b);
            bool can_connect_to_goal(int tree);
            bool isEdge(Eigen::VectorXd point, int tree);
            Eigen::VectorXd getEdge(Eigen::VectorXd point, int tree);
            std::vector<Eigen::VectorXd> reconstruct_path(int tree, Eigen::VectorXd x_init, Eigen::VectorXd x_goal);
            bool check_none_vector(Eigen::VectorXd  vector);
            void setEdge(Eigen::VectorXd key, Eigen::VectorXd value, int tree);
            void connect_to_the_goal(int tree);
            std::vector<Eigen::VectorXd> get_path();
            bool check_solution(std::vector<Eigen::VectorXd>& path);
            int sizeOfEdge(int tree);
            void printEdge(int tree);
            bool is_equal_vectors(Eigen::VectorXd a, Eigen::VectorXd b);  
            float cost_to_go(Eigen::VectorXd a, Eigen::VectorXd b);
            float path_cost(Eigen::VectorXd a, Eigen::VectorXd b, std::map<std::tuple<float, float, float>, Eigen::VectorXd> edges);
            float segment_cost(Eigen::VectorXd a, Eigen::VectorXd b);  
        };
    }
}
#endif