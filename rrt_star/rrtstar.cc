#include "rrtstar.h"

namespace kamaz {
namespace hagen {
    RRTStar::RRTStar(SearchSpace search_space, std::vector<Eigen::VectorXd> lengths_of_edges
                , Eigen::VectorXd start_pose, Eigen::VectorXd goal_pose, int _max_samples
                , int resolution, float pro, int _rewrite_count) : RRT(search_space, lengths_of_edges, start_pose, goal_pose, _max_samples, resolution, pro){
                    
            rewrite_count = _rewrite_count;
            c_best = std::numeric_limits<float>::infinity();
    }
    
    std::vector<std::tuple<float, Eigen::VectorXd>> RRTStar::get_nearby_vertices(int tree, Eigen::VectorXd x_init, Eigen::VectorXd x_new){
        auto X_near = nearby(tree, x_new, current_rewrite_count(tree));
        std::cout<< "RRTStar::get_nearby_vertices: size: " << X_near.size() << std::endl;
        std::cout<< "RRTStar::get_nearby_vertices: x_new: " << x_new.transpose() << std::endl;
       
        std::vector<std::tuple<float, Eigen::VectorXd>> L_near;
        for(auto const x_near : X_near){
            auto new_s = segment_cost(x_near, x_new);
            auto cost = path_cost(x_init, x_near, trees[tree].E) + new_s;
            //std::cout<< "RRTStar::get_nearby_vertices: cost"<< cost << std::endl;
            auto pose = x_near;
            // std::cout<< "-----44" << std::endl;
            std::tuple<float, Eigen::VectorXd> a(cost, pose);
            L_near.push_back(a);
        }
        // std::cout<< "-----45" << std::endl;
        std::sort(L_near.begin(), L_near.end(),
            [](const std::tuple<float, Eigen::VectorXd> a, const std::tuple<float, Eigen::VectorXd> b){
                return std::get<0>(a) < std::get<0>(b); 
        });
        return L_near;
    }

    int RRTStar::current_rewrite_count(int tree){
        if(rewrite_count == 0){
            return trees[tree].v_count;
        }
        std::cout<< "RRTStar::current_rewrite_count: "<< trees[tree].v_count << std::endl;
        return std::min(trees[tree].v_count, rewrite_count);
    }

    void RRTStar::rewrite(int tree, Eigen::VectorXd x_new, std::vector<std::tuple<float, Eigen::VectorXd>> L_near){
        for (auto const l_near : L_near){
            // auto c_near = std::get<0>(l_near);
            auto x_near = std::get<1>(l_near);
            auto curr_cost = path_cost(x_init, x_near, trees[tree].E);
            auto tent_cost = path_cost(x_init, x_new, trees[tree].E) + segment_cost(x_new, x_near);
            if((tent_cost < curr_cost) && (X.collision_free(x_near, x_new, r))){
                // trees[tree].E[x_near] = x_new;
                setEdge(x_near, x_new, tree);
            }
        }
    }

    void RRTStar::connect_shortest_valid(int tree, Eigen::VectorXd x_new, std::vector<std::tuple<float, Eigen::VectorXd>> L_near){
        std::cout<< "RRTStar::connect_shortest_valid : L_near size: "<< L_near.size() << std::endl;
        for (auto const l_near : L_near){
            auto c_near = std::get<0>(l_near);
            std::cout<<"c_near: " << c_near << std::endl;
            auto x_near = std::get<1>(l_near);
            auto cost_go = cost_to_go(x_near, x_goal);
            std::cout<<"cost_go: " << cost_go << std::endl;
            auto is_connected = connect_to_point(tree, x_near, x_new);
            
            if((c_near+cost_go < c_best) && is_connected){
                break;
            }
        }
    }

    std::vector<Eigen::VectorXd> RRTStar::rrt_star(){
        add_vertex(0, x_init);
        Eigen::VectorXd none_pose(3);
        none_pose << -1, -1, -1;
        // std::cout<< "-----1" << std::endl;
        add_edge(0, x_init, none_pose);
        std::vector<Eigen::VectorXd> path;
        // std::cout<< "-----2" << std::endl;
        while(true){
            for(auto const q : Q){
                for(int i=0; i<q[1]; i++){
                //    std::cout<< "-----3" << std::endl;
                   auto new_and_next = new_and_near(0, q);
                   if(new_and_next.size()==0){
                       continue;
                   }
                   auto x_new = new_and_next[0];
                   if(check_none_vector(x_new)){
                       continue;
                   }
                   std::cout << "RRTStar::rrt_star: sizeOfEdge: " << sizeOfEdge(0) << std::endl;
                //    std::cout<< "-----4" << std::endl;
                   auto L_near = get_nearby_vertices(0, x_init, x_new);
                   std::cout<< "RRTStar::rrt_star: L_near"<< L_near.size() << std::endl;
                   connect_shortest_valid(0, x_new, L_near);
                //    std::cout<< "-----6" << std::endl;
                   if (isEdge(x_new, 0)){
                       rewrite(0, x_new, L_near);
                   }
                //    std::cout<< "----7" << std::endl;
                   
                   auto solution = check_solution(path);
                   if(solution){
                       std::cout<< "number of trees: " << path.size()<< std::endl;
                       return path;
                   }
                }
            }
        }
    }    
} 
}