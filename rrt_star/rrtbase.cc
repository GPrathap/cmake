#include "rrtbase.h"

namespace kamaz {
namespace hagen {

    RRTBase::RRTBase(SearchSpace search_space, std::vector<Eigen::VectorXd> lengths_of_edges
                , Eigen::VectorXd start_pose, Eigen::VectorXd goal_pose, int _max_samples
                , int resolution, float pro){

        X = search_space;
        sample_taken = 0;
        max_samples = _max_samples;
        Q = lengths_of_edges;
        r = resolution;
        prc = pro;
        x_init = start_pose;
        x_goal = goal_pose;
        add_tree();
    }

    void RRTBase::add_tree(){
        Tree tree;
        tree.init(X);
        trees[0] = tree;
    }

    void RRTBase::add_vertex(int tree, Eigen::VectorXd v){
        trees[tree].V.insert(v);
        trees[tree].v_count += 1;
        sample_taken += 1;
    }

    void RRTBase::add_edge(int tree, Eigen::VectorXd child, Eigen::VectorXd parent){
        std::tuple<float, float, float> child_(child[0], child[1], child[2]);
        trees[tree].E[child_] = parent;
    }

    void RRTBase::printEdge(int tree){
        std::cout<<"RRTBase::printEdge===="<< std::endl;
        
        for(auto const&item : trees[tree].E){
            std::cout<< item.second.transpose() << std::endl;
        }
        std::cout<<"RRTBase::printEdge===="<< std::endl;
    }

    std::vector<Eigen::VectorXd> RRTBase::nearby(int tree, Eigen::VectorXd x, int max_neighbors){
        return trees[tree].V.nearest(x, max_neighbors);
    }

    Eigen::VectorXd RRTBase::get_nearest(int tree, Eigen::VectorXd x){
        auto neighbours = nearby(tree, x, 1);

        // std::cout<< "RRTBase::get_nearest: size: " << neighbours.size() << std::endl;  
        // std::cout<< "RRTBase::get_nearest: " << x.transpose() << std::endl;  
        // std::cout<< "RRTBase::get_nearest:neighbours " << neighbours.size() << std::endl;  
        
        // for(auto const neigh : neighbours){
        //     std::cout<< neigh.transpose() << std::endl;
        // }
        if(neighbours.size()==0){
            std::cout<< "there is no any neighbors" << std::endl;
            Eigen::VectorXd temp(3);
            temp << -1, -1, -1;
            return temp;
        }
        // std::cout<< "RRTBase::get_nearest: zero: " << neighbours[0].transpose() << std::endl; 
        return neighbours[0];  
    }

    bool RRTBase::check_none_vector(Eigen::VectorXd  vector){
        return (vector[0]<0)? true: false;
    }

    std::vector<Eigen::VectorXd> RRTBase::new_and_near(int tree, Eigen::VectorXd q){
        std::vector<Eigen::VectorXd> new_and_near_vec;
        auto x_rand = X.sample_free();
        auto x_nearest = get_nearest(tree, x_rand);
        auto x_new = steer(x_nearest, x_rand, q[0]);
        std::cout<<"RRTBase::new_and_near: x_rand: " << x_rand.transpose() << std::endl;
        std::cout<<"RRTBase::new_and_near: x_nearest: " << x_nearest.transpose() << std::endl;
        // std::cout<<"RRTBase::new_and_near: x_new " << x_new.transpose() << std::endl;
        printEdge(0);
        auto g1 = trees[0].V.obstacle_free(x_new);
        auto g2 = X.obstacle_free(x_new);
        std::cout<<"RRTBase::new_and_near: x_new g1 g2 " << g1 << "  "<< g2 << std::endl;
        if((!g1) && (!g2)){
            return new_and_near_vec;
        }
        sample_taken += 1;
        new_and_near_vec.push_back(x_new);
        new_and_near_vec.push_back(x_nearest);
        std::cout<<"RRTBase::new_and_near: new_and_near_vec "<< new_and_near_vec.size() <<std::endl;
        return new_and_near_vec;
    }

    Eigen::VectorXd RRTBase::steer(Eigen::VectorXd start, Eigen::VectorXd goal, float distance){
        auto ab = goal - start;
        Eigen::VectorXd zero_vector(3);
        zero_vector << 0, 0, 0 ;
        auto ba_length = (ab - zero_vector).norm();
        auto unit_vector = ab/ba_length;
        auto scaled_vector = unit_vector*distance;
        Eigen::VectorXd steered_point = start + scaled_vector;
        int j = 0;
        for(int i=0; i<6; i+=2){
            if(steered_point[j] < X.dim_lengths[i]){
                steered_point[j] = X.dim_lengths[i];
            }
            if (steered_point[j] >= X.dim_lengths[i+1]){
                steered_point[j] = X.dim_lengths[i+1];
            }
            j++;
        }
        return steered_point;
    }

    bool RRTBase::connect_to_point(int tree, Eigen::VectorXd x_a, Eigen::VectorXd x_b){
        std::cout<< "RRTBase::connect_to_point: "<< x_b.transpose() << std::endl;
        std::cout<< "RRTBase::connect_to_point: "<< x_a.transpose() << std::endl;

        auto g1 = trees[tree].V.obstacle_free(x_b);
        auto g2 = X.collision_free(x_a, x_b, r);
        
        std::cout<< "RRTBase::connect_to_point: "<< g1 << " " << g2 << std::endl;

        if(( g1 == 1) && ( g2 == 1)){
            add_vertex(tree, x_b);
            add_edge(tree, x_b, x_a);
            return true;
        }
        std::cout<< "RRTBase::connect_to_point not connected" << std::endl;
        return false;
    }

    bool RRTBase::can_connect_to_goal(int tree){
        auto x_nearest = get_nearest(tree, x_goal);
        std::cout<< "RRTBase::can_connect_to_goal:x_nearest: "<< x_nearest.transpose() << std::endl;
        std::cout<< "RRTBase::can_connect_to_goal:x_goal: "<< x_goal.transpose() << std::endl;

        auto f1 = isEdge(x_goal, tree);
        auto f2 = isEdge(x_nearest, tree);

        std::cout<< "RRTBase::can_connect_to_goal: "<< f1 << " "<< f2 << " " << std::endl;
       
        if( f1 && f2){
            return true;
        }
        if(X.collision_free(x_nearest, x_goal, r)){
             std::cout<< "RRTBase::can_connect_to_goal: collision_free true"<< std::endl;
            return true;
        }
        return false;
    }

    void RRTBase::connect_to_the_goal(int tree){
        auto x_nearest = get_nearest(tree, x_goal);
        setEdge(x_goal, x_nearest, tree);
    }

    std::vector<Eigen::VectorXd> RRTBase::get_path(){
        std::vector<Eigen::VectorXd> path;
        if(can_connect_to_goal(0)){
            std::cout << "Can connect to goal" << std::endl;
            connect_to_the_goal(0);
            return reconstruct_path(0, x_init, x_goal);
        }
        std::cout << "Could not connect to goal" << std::endl;
        return path;
    }

    bool RRTBase::isEdge(Eigen::VectorXd point, int tree){
        std::tuple<float, float, float> _key(point[0], point[1], point[2]);
        return (trees[tree].E.count(_key)) > 0 ? true : false;
    }

    Eigen::VectorXd RRTBase::getEdge(Eigen::VectorXd point, int tree){
        std::tuple<float, float, float> _key(point[0], point[1], point[2]);
        return trees[tree].E[_key];
    }

    void RRTBase::setEdge(Eigen::VectorXd key, Eigen::VectorXd value, int tree){
        std::tuple<float, float, float> _key(key[0], key[1], key[2]);
        trees[tree].E[_key] = value;
    }

    int RRTBase::sizeOfEdge(int tree){
        return trees[tree].E.size();
    }

    std::vector<Eigen::VectorXd> RRTBase::reconstruct_path(int tree, Eigen::VectorXd x_init, Eigen::VectorXd x_goal){
        std::vector<Eigen::VectorXd> path;
        path.push_back(x_goal);
        auto current = x_goal;
        
        std::cout<< "RRTBase::reconstruct_path: current"<< current.transpose() << std::endl;
        std::cout<< "RRTBase::reconstruct_path: current"<< current.transpose() << std::endl;
        if(is_equal_vectors(x_goal, x_init)){
            return path;
        }
        // printEdge(tree);
        if(isEdge(current, tree)){
            auto current_parent = getEdge(current, tree);
            // std::cout<< "RRTBase::reconstruct_path: current"<< current_parent.transpose() << std::endl;
            while(!is_equal_vectors(current_parent, x_init)){
                path.push_back(current_parent);
                std::cout<< "RRTBase::reconstruct_path: current"<< current_parent.transpose() << std::endl;
                current_parent = getEdge(current_parent, tree);
            }
            path.push_back(x_init);
            std::reverse(path.begin(), path.end());
        }else{
            std::cout<< "Something wrong with map, need to change it"<< std::endl;
        }

        std::cout<< "RRTBase::reconstruct_path: size_of the path: "<< path.size() << std::endl;
        return path; 
    }

    bool RRTBase::is_equal_vectors(Eigen::VectorXd a, Eigen::VectorXd b){
        return ((a - b).norm()< min_acceptable) ? true : false;
    }

    bool RRTBase::check_solution(std::vector<Eigen::VectorXd>& path){
        if ( prc > std::rand() %1 ){
            std::cout<< "Checking if can connect to the goal at "<< sample_taken << " samples" << std::endl;
            path = get_path();
            if(path.size()>0){
                return true;
            }
        }

        if(sample_taken >= max_samples){
           path = get_path();
           return true; 
        }
        return false;
    }

    float RRTBase::cost_to_go(Eigen::VectorXd a, Eigen::VectorXd b){
        return (a-b).norm();
    }

    float RRTBase::path_cost(Eigen::VectorXd a, Eigen::VectorXd b, std::map<std::tuple<float, float, float>, Eigen::VectorXd> edges){
        float cost = 0;
        // std::cout<< "RRTBase::path_cost" << std::endl;
        // std::cout<< "RRTBase::path_cost: a" << a.transpose() << std::endl;
        // std::cout<< "RRTBase::path_cost: b" << b.transpose() << std::endl;
        while(!is_equal_vectors(a, b)){
            // std::cout<< "RRTBase::path_cost:a "<< a.transpose() << "  b: "<< b.transpose() << std::endl;
            std::tuple<float, float, float> _key(b[0], b[1], b[2]);
            if(edges.count(_key)<1){
                // std::cout<< "RRTBase::path_cost:empty edge " << std::endl;
                break;
            }
            auto p = edges[_key];
            // std::cout<< "-----423:"<< p.transpose() << std::endl;
            cost += (b-p).norm();
            // std::cout<< "-----424"<< a << "  "<< b << std::endl;
            b = p;
        }
        // std::cout<< "RRTBase::path_cost: cost: " << cost << std::endl;
        return cost;
    }

    float RRTBase::segment_cost(Eigen::VectorXd a, Eigen::VectorXd b){
        return (a-b).norm();
    }

}
}