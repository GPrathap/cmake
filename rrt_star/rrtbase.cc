#include "rrtbase.h"

namespace kamaz {
namespace hagen {

    RRTBase::RRTBase(RRTPlannerOptions options, CommonUtils& common_util
    , std::atomic_bool &is_allowed_to_run): till_auto_mode(is_allowed_to_run){
            
            X = options.search_space;
            Q = options.lengths_of_edges;
            sample_taken = 0;
            max_samples = options.max_samples;
            r = options.resolution;
            prc = options.pro;
            x_init = options.x_init;
            x_goal = options.x_goal;
            start_position = options.start_position;
            min_angle_allows_obs = options.min_angle_allows_obs;
            add_tree();
            _obstacle_fail_safe_distance = options.obstacle_fail_safe_distance;
            common_utils = common_util;
    }

    void RRTBase::add_tree(){
        Tree tree;
        tree.init(X);
        trees[0] = tree;
    }

    void RRTBase::add_vertex(int tree, Eigen::VectorXf v){
        trees[tree].V.insert_vertex(v);
        trees[tree].v_count += 1;
        sample_taken += 1;
    }

    void RRTBase::add_edge(int tree, Eigen::VectorXf child, Eigen::VectorXf parent){
        std::array<float, 3> child_ = {child[0], child[1], child[2]};
        trees[tree].E[child_] = parent;
    }

    void RRTBase::printEdge(int tree){
        std::cout<<"RRTBase::printEdge===="<< std::endl;
        for(auto const&item : trees[tree].E){
            std::cout<<item.first[0] << "," << item.first[1] 
                                << "," << item.first[2] << std::endl;
            std::cout<< item.second.transpose() << std::endl;
            std::cout<< "=============" << std::endl;
        }
        std::cout<<"RRTBase::printEdge===="<< std::endl;
    }

    std::vector<Eigen::VectorXf> RRTBase::nearby_obstacles(int tree, Eigen::VectorXf x
                                            , int max_neighbors){
        return trees[tree].V.nearest_obstacles(x, max_neighbors);
    }

    std::vector<Eigen::VectorXf> RRTBase::nearby_waypoints(int tree, Eigen::VectorXf x
                                            , int max_neighbors){
        return trees[tree].V.nearest_point_on_trajectory(x, max_neighbors);
    }

    Eigen::VectorXf RRTBase::get_nearest(int tree, Eigen::VectorXf x){
        auto veties = trees[tree].V.nearest_veties(x, 1);

        // auto neighbour_obstacles = nearby_obstacles(tree, x, 1);
        // // std::cout<< "RRTBase::get_nearest: size: " << neighbours.size() << std::endl;  
        // // std::cout<< "RRTBase::get_nearest: " << x.transpose() << std::endl;  
        // // std::cout<< "RRTBase::get_nearest:neighbours " << neighbours.size() << std::endl;  
        
        // // for(auto const neigh : neighbours){
        // //     std::cout<< neigh.transpose() << std::endl;
        // // }
        if(veties.size()==0){
            BOOST_LOG_TRIVIAL(warning) << FYEL("There is no any neighbors");
            Eigen::VectorXf temp(3);
            temp << -1, -1, -1;
            return temp;
        }
        return veties[0];
        //TODO remove this later
        // if(X.use_whole_search_sapce){
        //     return neighbour_obstacles[0];  
        // }
        // auto nearest_point_to_trajectory = nearby_waypoints(tree, neighbour_obstacles[0], 1);
        // if(nearest_point_to_trajectory.size() > 0){
        //     std::cout<< "nearest point on trajectory:" << nearest_point_to_trajectory[0].transpose() << std::endl;
        //     std::cout<< "neighbour_obstacle:" << neighbour_obstacles[0].transpose() << std::endl;
        //     float d_s = (x.head(3) - nearest_point_to_trajectory[0].head(3)).norm();
        //     float d_o = (neighbour_obstacles[0].head(3) - nearest_point_to_trajectory[0].head(3)).norm();
        //     std::cout<< "distance to random point: " << d_s << std::endl;
        //     std::cout<< "distance to obstacle: "<< d_o << std::endl;
            
        //     int size_of_samples = 10;
        //     center_sub_search =  nearest_point_to_trajectory[0].head(3);
        //     covmat_sub_search = Eigen::MatrixXf::Zero(3,3);
        //     float radius = d_o;
            // if(d_o == 0){
            //     // TODO need to fix this
            //     return neighbour_obstacles[0];  
            // }
            
            // radius = 60.0;
            // covmat_sub_search(0,0) = radius;
            // covmat_sub_search(1,1) = radius;
            // covmat_sub_search(2,2) = radius;
            // // std::cout<< "covmat_sub_search" << std::endl;
            // // std::cout<< covmat_sub_search << std::endl;
            // Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity(3,3);
            // int ndims = covmat_sub_search.rows();       
            // Eigen::MatrixXf random_points = Eigen::MatrixXf::Zero(size_of_samples, ndims);
            // common_utils.generate_samples_from_ellipsoid(covmat_sub_search, rotation_matrix, center_sub_search
            // , random_points);
            // int max_attemp = 2;
            // int attemp = 0;
            // while(attemp <= max_attemp){
            //     for(int inx=0; inx < size_of_samples; inx++){
            //         Eigen::VectorXf next_pose = random_points.row(inx);
            //         float dis = (next_pose-neighbour_obstacles[0].head(3)).norm();
            //         // std::cout << "Generated sample: "<< next_pose.transpose() << std::endl;
                    
            //         float next_pose_norm = next_pose.head(3).norm();
            //         float obs_norm = neighbour_obstacles[0].head(3).norm();
            //         if(next_pose_norm <= 0.5 || obs_norm <= 0.5){
            //             continue;
            //         }
            //         float angle_diff = next_pose.head(3).dot(neighbour_obstacles[0].head(3))/(next_pose_norm*obs_norm);
            //         angle_diff = std::abs(std::acos(angle_diff)*(180.0/3.141));
            //         std::cout << "Angle=====================----" << angle_diff << std::endl;
            //         if((dis>_obstacle_fail_safe_distance) && (angle_diff > min_angle_allows_obs)){
            //             if(X.obstacle_free(next_pose)){
            //                 std::cout << "Valid generated sample:  "
            //                 << next_pose.transpose() << " at " << attemp << " attemp" << std::endl;
            //                 Eigen::VectorXf path_position(4);
            //                 std::cout<< "*current_position" << (start_position).transpose() << std::endl;
                            
            //                 // common_utils.get_point_on_the_trajectory(center_sub_search
            //                 // , start_position , path_position);
                            
            //                 // std::cout<< "*current_position" << (path_position).transpose() << std::endl;
            //                 return next_pose;
            //             }
            //         }
            //     }
            //     covmat_sub_search(0,0) = radius*2;
            //     covmat_sub_search(1,1) = radius*2;
            //     covmat_sub_search(2,2) = radius;
            //     attemp = attemp + 1 ;
            // }
            
        // }
        // return neighbour_obstacles[0];  
    }

    bool RRTBase::check_none_vector(Eigen::VectorXf  vector){
        return (vector[0]<0)? true: false;
    }

    std::vector<Eigen::VectorXf> RRTBase::new_and_near(int tree, Eigen::VectorXf q){
        std::vector<Eigen::VectorXf> new_and_near_vec;
        auto x_rand = X.sample_free();
        auto x_nearest = get_nearest(tree, x_rand);
        auto x_new = steer(x_nearest, x_rand, q[0]);
        // std::cout<<"RRTBase::new_and_near: x_rand: " << x_rand.transpose() << std::endl;
        // std::cout<<"RRTBase::new_and_near: x_nearest: " << x_nearest.transpose() << std::endl;
        // std::cout<<"RRTBase::new_and_near: x_new " << x_new.transpose() << std::endl;
        // printEdge(0);
        auto g1 = trees[0].V.obstacle_free(x_new);
        auto g2 = X.obstacle_free(x_new);
        // std::cout<<"RRTBase::new_and_near: x_new g1 g2 " << g1 << "  "<< g2 << std::endl;
        if((!g1) && (!g2)){
            return new_and_near_vec;
        }
        sample_taken += 1;
        new_and_near_vec.push_back(x_new);
        new_and_near_vec.push_back(x_nearest);
        // std::cout<<"RRTBase::new_and_near: new_and_near_vec "<< new_and_near_vec.size() <<std::endl;
        return new_and_near_vec;
    }

    Eigen::VectorXf RRTBase::steer(Eigen::VectorXf start, Eigen::VectorXf goal, float distance){
        auto ab = goal - start;
        auto ba_length = ab.norm();
        auto unit_vector = ab/ba_length;
        auto scaled_vector = unit_vector*distance;
        Eigen::VectorXf steered_point = start + scaled_vector;
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

    bool RRTBase::connect_to_point(int tree, Eigen::VectorXf x_a, Eigen::VectorXf x_b){
        // std::cout<< "RRTBase::connect_to_point: "<< x_b.transpose() << std::endl;
        // std::cout<< "RRTBase::connect_to_point: "<< x_a.transpose() << std::endl;

        auto g1 = trees[tree].V.obstacle_free(x_b);
        auto g2 = X.collision_free(x_a, x_b, r);
        
        // std::cout<< "RRTBase::connect_to_point: "<< g1 << " " << g2 << std::endl;

        if(( g1 == 1) && ( g2 == 1)){
            add_vertex(tree, x_b);
            add_edge(tree, x_b, x_a);
            return true;
        }
        // std::cout<< "RRTBase::connect_to_point not connected" << std::endl;
        return false;
    }

    bool RRTBase::can_connect_to_goal(int tree){
        auto x_nearest = get_nearest(tree, x_goal);
        // std::cout<< "RRTBase::can_connect_to_goal:x_nearest: "<< x_nearest.transpose() << std::endl;
        // std::cout<< "RRTBase::can_connect_to_goal:x_goal: "<< x_goal.transpose() << std::endl;

        auto f1 = isEdge(x_goal, tree);
        auto f2 = isEdge(x_nearest, tree);

        // std::cout<< "RRTBase::can_connect_to_goal: "<< f1 << " "<< f2 << " " << std::endl;
       
        if( f1 && f2){
            return true;
        }
        if(X.collision_free(x_nearest, x_goal, r)){
            //  std::cout<< "RRTBase::can_connect_to_goal: collision_free true"<< std::endl;
            return true;
        }
        return false;
    }

    void RRTBase::connect_to_the_goal(int tree){
        auto x_nearest = get_nearest(tree, x_goal);
        // std::cout<< "RRTBase::connect_to_the_goal" << std::endl;
        // std::cout<< x_nearest.transpose() << std::endl;
        setEdge(x_goal, x_nearest, tree);
    }

    std::vector<Eigen::VectorXf> RRTBase::get_path(){
        std::vector<Eigen::VectorXf> path;
        if(can_connect_to_goal(0)){
             BOOST_LOG_TRIVIAL(info) << FCYN("Can connect to goal");
            connect_to_the_goal(0);
            return reconstruct_path(0, x_init, x_goal);
        }
         BOOST_LOG_TRIVIAL(info) << FCYN("Could not connect to goal");
        return path;
    }

    bool RRTBase::isEdge(Eigen::VectorXf point, int tree){
        std::array<float, 3> _key = {point[0], point[1], point[2]};
        return (trees[tree].E.count(_key)) > 0 ? true : false;
    }

    Eigen::VectorXf RRTBase::getEdge(Eigen::VectorXf point, int tree){
        std::array<float, 3> _key = {point[0], point[1], point[2]};
        return trees[tree].E[_key];
    }

    void RRTBase::setEdge(Eigen::VectorXf key, Eigen::VectorXf value, int tree){
        std::array<float, 3> _key = {key[0], key[1], key[2]};
        trees[tree].E[_key] = value;
    }

    int RRTBase::sizeOfEdge(int tree){
        return trees[tree].E.size();
    }

    std::vector<Eigen::VectorXf> RRTBase::reconstruct_path(int tree, Eigen::VectorXf x_init, Eigen::VectorXf x_goal){
        std::vector<Eigen::VectorXf> path;
        path.push_back(x_goal);
        auto current = x_goal;
        
        // std::cout<< "RRTBase::reconstruct_path: current"<< current.transpose() << std::endl;
        // std::cout<< "RRTBase::reconstruct_path: current"<< current.transpose() << std::endl;
        if(is_equal_vectors(x_goal, x_init)){
            return path;
        }
        // printEdge(tree);
        if(isEdge(current, tree)){
            auto current_parent = getEdge(current, tree);
            // std::cout<< "RRTBase::reconstruct_path: current 1"<< current_parent.transpose() << std::endl;
            while(!is_equal_vectors(current_parent, x_init)){
                path.push_back(current_parent);
                // std::cout<< "RRTBase::reconstruct_path: current 2"<< current_parent.transpose() << std::endl;

                if(isEdge(current_parent, tree)){
                    current_parent = getEdge(current_parent, tree);
                    // std::cout<< "RRTBase::reconstruct_path: current is edge 3"<< current_parent.transpose() << std::endl;
                }else{
                    // std::cout<< "RRTBase::reconstruct_path: current: something wrong with edges"<< current_parent.transpose() << std::endl;
                    // return path;
                    break;
                }
            }
            path.push_back(x_init);
            std::reverse(path.begin(), path.end());
        }else{
            BOOST_LOG_TRIVIAL(fatal) << FRED("Something wrong with map, need to change it");
        }

        BOOST_LOG_TRIVIAL(info) << FCYN("RRTBase::reconstruct_path: size_of the path: ")<< path.size();
        return path; 
    }

    bool RRTBase::is_equal_vectors(Eigen::VectorXf a, Eigen::VectorXf b){
        return ((a - b).norm()< min_acceptable) ? true : false;
    }

    bool RRTBase::check_solution(std::vector<Eigen::VectorXf>& path){
        if ( prc > std::rand() %1 ){
            BOOST_LOG_TRIVIAL(info) << FCYN("Checking if can connect to the goal at ")<< sample_taken << FCYN(" samples");
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

    float RRTBase::cost_to_go(Eigen::VectorXf a, Eigen::VectorXf b){
        return (a-b).norm();
    }

    float RRTBase::path_cost(Eigen::VectorXf a, Eigen::VectorXf b, int tree){
        float cost = 0;
        // std::cout<< "RRTBase::path_cost" << std::endl;
        // std::cout<< "RRTBase::path_cost: a" << a.transpose() << std::endl;
        // std::cout<< "RRTBase::path_cost: b" << b.transpose() << std::endl;
        auto edges = trees[tree].E;
        while(!is_equal_vectors(a, b)){
            // std::cout<< "RRTBase::path_cost:a "<< a.transpose() << "  b: "<< b.transpose() << std::endl;
            std::array<float, 3> _key = {b[0], b[1], b[2]};
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

    float RRTBase::segment_cost(Eigen::VectorXf a, Eigen::VectorXf b){
        return (a-b).norm();
    }

}
}