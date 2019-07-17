#include "rrtconnect.h"

namespace kamaz {
namespace hagen {

    RRTConnect::RRTConnect(SearchSpace search_space, std::vector<Eigen::VectorXf> lengths_of_edges
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose
                , Eigen::VectorXf ostacle_pose, int _max_samples
                , int resolution, float pro)
        :RRTBase(search_space, lengths_of_edges, start_pose, goal_pose, ostacle_pose, _max_samples, resolution, pro){
            swapped = false;
    }

    void RRTConnect::swap_trees(){
        auto default_tree = trees[0];
        trees[0] = trees[1];
        trees[1] = default_tree;
        swapped = !swapped;
    }

    void RRTConnect::unswap(){
        if (swapped){
            swap_trees();
        }
    }

    RRTStatus RRTConnect::extend(int tree, Eigen::VectorXf x_rand, Eigen::VectorXf& x_final){
        auto x_nearest = get_nearest(tree, x_rand);
        auto x_new = steer(x_nearest, x_rand, Q[0][0]);
        x_final = x_new;
        if(connect_to_point(tree, x_nearest, x_new)){
            if(is_equal_vectors(x_new, x_rand)){
                return RRTStatus::REACHED;
            }
            return RRTStatus::ADVANCED;
        }
        return RRTStatus::TRAPPED;
    } 

    RRTStatus RRTConnect::connect(int tree, Eigen::VectorXf x, Eigen::VectorXf x_connect){
        auto S = RRTStatus::ADVANCED;
        Eigen::VectorXf x_new(3);
        while(S == RRTStatus::ADVANCED){
            S = extend(tree, x, x_new);
        }
        x_connect = x_new;
        return S;
    }

    std::vector<Eigen::VectorXf> RRTConnect::rrt_connect(){
        add_vertex(0, x_init);
        Eigen::VectorXf none_pose(3);
        none_pose << -1, -1, -1;
        add_edge(0, x_init, none_pose);
        add_tree();
        add_vertex(1, x_goal);
        add_edge(1, x_goal, none_pose);
        while(sample_taken < max_samples){
            auto x_rand = X.sample_free();
            Eigen::VectorXf x_new(3);
            auto status = extend(0, x_rand, x_new);
            if(status != RRTStatus::TRAPPED){
                Eigen::VectorXf x_new_(3);
                auto connect_status = connect(1, x_new, x_new_);
                if(connect_status == RRTStatus::REACHED){
                    unswap();
                    auto first_path = reconstruct_path(0, x_init, get_nearest(0, x_new_));
                    auto secound_path = reconstruct_path(1, x_goal, get_nearest(1, x_new_));
                    std::reverse(secound_path.begin(), secound_path.end());
                    first_path.insert(first_path.end(), secound_path.begin(), secound_path.end());
                    return first_path;
                }
            }
            swap_trees();
            sample_taken += 1;
        }
    }


}
}