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
#include <atomic>
#include "../common/search_space.h"
#include "../utils/common_utils.h"
#include "tree.h"
#include <unordered_map>
#include <stddef.h>

namespace kamaz {
namespace hagen {
        struct PathNode
        {
            public:
            /* -------------------- */
            Eigen::Vector3d index;
            Eigen::Matrix<double, 6, 1> state;
            double g_score, f_score;
            Eigen::Vector3d input;
            double duration;
            double time;  // dyn
            int time_idx;
            PathNode* parent;
            char node_state;
            PathNode()
            {
                parent = NULL;
                node_state = 0;
            }
            ~PathNode(){};
        };
        typedef PathNode* PathNodePtr;
        struct RRTPlannerOptions {
            SearchSpace search_space;
            int sample_taken;
            int max_samples;
            std::vector<Eigen::Vector2d> lengths_of_edges;
            int resolution;
            double pro;
            double voxel_side_length;
            Eigen::Vector3d init_min_point;
            Eigen::Vector3d x_init;
            Eigen::Vector3d x_goal;
            Eigen::Vector3d start_position;
            double obstacle_fail_safe_distance;
            double min_acceptable;
            double min_angle_allows_obs;
        };
        class RRTBase {
            public:
                

                RRTBase(RRTPlannerOptions options, 
                    CommonUtils& common_util, std::atomic_bool &is_allowed_to_run);
                ~RRTBase() = default; 

                SearchSpace X;
                CommonUtils common_utils;
                int sample_taken;
                int max_samples;
                std::vector<Eigen::Vector2d> Q;
                int r;
                double prc;
                double voxel_side_length;
                Eigen::Vector3d init_min_point;
                std::atomic_bool& till_auto_mode;
                Eigen::Vector3d x_init;
                Eigen::Vector3d x_goal;
                Eigen::Vector3d start_position;
                std::map<int, Tree> trees;
                Eigen::MatrixXd covmat_sub_search;
                Eigen::Vector3d center_sub_search;
                double _obstacle_fail_safe_distance = 0.5f;
                double min_acceptable = 0.01;
                double min_angle_allows_obs = 5.0f;

                void add_tree();
                void add_vertex(int tree, Eigen::Vector3d v);
                void add_edge(int tree, Eigen::Vector3d child, Eigen::Vector3d parent);
                std::vector<Eigen::Vector3d> nearby_vertices(int tree, Eigen::Vector3d x
                                , int max_neighbors);
                std::vector<Eigen::Vector3d> nearby_waypoints(int tree, Eigen::Vector3d x
                                , int max_neighbors);
                Eigen::Vector3d get_nearest(int tree, Eigen::Vector3d x);
                Eigen::Vector3d steer(Eigen::Vector3d start, Eigen::Vector3d goal
                                    , double distance);
                std::vector<Eigen::Vector3d> new_and_near(int tree, Eigen::Vector2d q);
                bool connect_to_point(int tree, Eigen::Vector3d x_a, Eigen::Vector3d x_b);
                bool can_connect_to_goal(int tree);
                bool isEdge(Eigen::Vector3d point, int tree);
                Eigen::Vector3d getEdge(Eigen::Vector3d point, int tree);
                std::vector<Eigen::Vector3d> reconstruct_path(int tree
                                        , Eigen::Vector3d x_init, Eigen::Vector3d x_goal);
                bool check_none_vector(Eigen::Vector3d  vector);
                void setEdge(Eigen::Vector3d key, Eigen::Vector3d value, int tree);
                void connect_to_the_goal(int tree);
                std::vector<Eigen::Vector3d> get_path();
                bool check_solution(std::vector<Eigen::Vector3d>& path);
                int sizeOfEdge(int tree);
                void printEdge(int tree);
                bool is_equal_vectors(Eigen::Vector3d a, Eigen::Vector3d b);
                double cost_to_go(Eigen::Vector3d a, Eigen::Vector3d b);
                double path_cost(Eigen::Vector3d a, Eigen::Vector3d b, int tree);
                double segment_cost(Eigen::Vector3d a, Eigen::Vector3d b);
                
        };
    }
}
#endif