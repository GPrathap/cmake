#include "rrt.h"

namespace kamaz {
namespace hagen {
    
    RRT::RRT(SearchSpace search_space, std::vector<Eigen::VectorXf> lengths_of_edges
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose, Eigen::VectorXf first_object_found_pose, int _max_samples
                , int resolution, float pro)
        :RRTBase(search_space, lengths_of_edges, start_pose, goal_pose
            , first_object_found_pose, _max_samples, resolution, pro){
            
        }
    
    std::vector<Eigen::VectorXf> RRT::rrt_search(){
        add_vertex(0, x_init);
        Eigen::VectorXf none_pose(3);
        none_pose << -1, -1, -1;
        add_edge(0, x_init, none_pose); //TODO need to handle this proper way setting null pointer
        std::vector<Eigen::VectorXf> path;
        while(true){
            for(auto const q : Q){
                for(int i=0; i<q[1]; i++){
                   auto new_and_next = new_and_near(0, q);
                   if(check_none_vector(new_and_next[0])){
                       continue;
                   }
                   connect_to_point(0, new_and_next[1], new_and_next[0]);
                   auto solution = check_solution(path);
                   if(solution){
                       return path;
                   }
                }
            }
        }
    }
}
}