#include "rrt_star_3d.h"

namespace kamaz {
namespace hagen {

    std::vector<Eigen::VectorXf> RRTStar3D::rrt_planner(SearchSpace search_space
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose
                , Eigen::VectorXf start_position, float obstacle_fail_safe_distance
                , float min_angle_allows_obs, CommonUtils& common_utils
                , std::atomic_bool &is_allowed_to_run){
        
        rrt_planner_options.search_space = search_space;
        rrt_planner_options.x_init = start_pose;
        rrt_planner_options.x_goal = goal_pose;
        rrt_planner_options.start_position = start_position;
        rrt_planner_options.obstacle_fail_safe_distance = obstacle_fail_safe_distance;
        rrt_planner_options.min_angle_allows_obs = min_angle_allows_obs;
        auto rrtstar = RRTStar(rrt_planner_options, _rewrite_count
                                            , common_utils, is_allowed_to_run);
        return rrtstar.rrt_star();
    }

    std::vector<Eigen::VectorXf> RRTStar3D::rrt_planner_and_save(SearchSpace search_space
                , Eigen::VectorXf start_pose, Eigen::VectorXf goal_pose
                , Eigen::VectorXf start_position, float obstacle_fail_safe_distance
                , float min_angle_allows_obs, CommonUtils& common_utils
                , std::atomic_bool &is_allowed_to_run, int index){
        
        rrt_planner_options.search_space = search_space;
        rrt_planner_options.x_init = start_pose;
        rrt_planner_options.x_goal = goal_pose;
        rrt_planner_options.start_position = start_position;
        rrt_planner_options.min_angle_allows_obs = min_angle_allows_obs;
        rrt_planner_options.obstacle_fail_safe_distance = obstacle_fail_safe_distance;

        auto rrtstar =  RRTStar(rrt_planner_options, _rewrite_count, common_utils
                        , is_allowed_to_run);
        
        std::ofstream outfile;
        outfile.open("/dataset/rrt_old/time_stamps.txt", std::ios_base::app);
        const clock_t begin_time = clock();
        auto path = rrtstar.rrt_star();
        float time_diff =  float( clock () - begin_time ) /  CLOCKS_PER_SEC;
        if(search_space.use_whole_search_sapce){
             outfile << "rrt,"<<  time_diff <<","<< path.size() << "," << get_distance(path) << "," <<  common_utils.get_cost_of_path(path) << "\n";
        }else{
            outfile << "rrt_modified,"<<  time_diff <<","<< path.size() << "," << get_distance(path) << ","<< common_utils.get_cost_of_path(path) << "\n";
        }
        stotage_location = "/dataset/rrt_old/"+ std::to_string(index) + "_";
        save_edges(rrtstar.trees, stotage_location + "edges.npy");
        save_obstacle(search_space.random_objects, stotage_location + "obstacles.npy");
        save_poses(start_pose, goal_pose, stotage_location + "start_and_end_pose.npy");
        save_path(path, stotage_location + "rrt_star_path.npy");
        return path;
    }

    float  RRTStar3D::get_distance(std::vector<Eigen::VectorXf> trajectory_){
			float distance = 0.0f;
        if(trajectory_.size() < 1){
            return distance;
        }
        Eigen::VectorXf previous = trajectory_[0].head(3);
        for (int i = 1; (unsigned)i < trajectory_.size(); i++){
            float dis = std::abs((previous.head(3) - trajectory_[i].head(3)).norm());
            previous = trajectory_[i].head(3);
            distance += dis;
        }
        return distance;
    }

    void RRTStar3D::save_trajectory(std::vector<Eigen::VectorXf> trajectory_of_drone){
       std::string file_name = stotage_location + "smoothed_rrt_path.npy";
       std::vector<double> quad_status; 
       for(auto sector: trajectory_of_drone){
            // std::cout<< status.size() << std::endl;
            quad_status.push_back(sector[0]);
            quad_status.push_back(sector[1]);
            quad_status.push_back(sector[2]);
       }
       cnpy::npy_save(file_name, &quad_status[0], {quad_status.size()}, "w");
    }

    void RRTStar3D::rrt_init(std::vector<Eigen::VectorXf> _lengths_of_edges
            , int max_samples, int _resolution, float _pro, int rewrite_count){
        rrt_planner_options.lengths_of_edges = _lengths_of_edges;
        rrt_planner_options.max_samples = max_samples;
        rrt_planner_options.resolution = _resolution; 
        rrt_planner_options.pro = _pro;
        _rewrite_count = rewrite_count;
    }

    Eigen::VectorXf RRTStar3D::get_search_space_dim(Eigen::VectorXf dimensions){
        Eigen::VectorXf dim_(6);
        dim_<< 0, dimensions[0], 0, dimensions[1], 0, dimensions[2];
        return dim_;
    }

    std::vector<SearchSpace::Rect> RRTStar3D::get_obstacles(){
        std::vector<SearchSpace::Rect> _objects;
        _objects.push_back(SearchSpace::Rect(20, 20, 20, 40, 40, 40));
        _objects.push_back(SearchSpace::Rect(20, 20, 60, 40, 40, 80));
        _objects.push_back(SearchSpace::Rect(20, 60, 20, 40, 80, 40));
        _objects.push_back(SearchSpace::Rect(60, 60, 20, 80, 80, 40));
        _objects.push_back(SearchSpace::Rect(60, 20, 20, 80, 40, 40));
        _objects.push_back(SearchSpace::Rect(60, 20, 60, 80, 40, 80));
        _objects.push_back(SearchSpace::Rect(20, 60, 60, 40, 80, 80));
        _objects.push_back(SearchSpace::Rect(60, 60, 60, 80, 80, 80));
        return _objects;
    }

    std::vector<SearchSpace::Rect> RRTStar3D::get_random_obstacles(int number_of_obstacles, Eigen::VectorXf x_dimentions){
        std::vector<SearchSpace::Rect> _objects;
        srand(time(NULL));
        for(int i=0; i< number_of_obstacles; i++){
            
            float x = (rand()%(int)x_dimentions[0]);
            float y = (rand()%(int)x_dimentions[1]);
            float z = (rand()%(int)x_dimentions[2]);
            int width_x = x + rand()%10;
            int width_y = y + rand()%10;
            int width_z = z + rand()%10;
            if(width_x >= (int)x_dimentions[0] 
            || width_y >= (int)x_dimentions[1] || width_z >= (int)x_dimentions[2]){
                continue;
            }
            _objects.push_back(SearchSpace::Rect(x, y, z, width_x, width_y, width_z));
        }
        // std::cout<< "RRTStar3D::get_random_obstacles: Size of the obbjects "<< _objects.size() << std::endl;
        return _objects;
    }

    void RRTStar3D::save_edges(std::map<int, Tree> trees, std::string file_name){
       std::vector<float> edges; 
       int count = 0;
       for(auto const&item : trees[0].E){
            // std::cout<< item.second.transpose() << std::endl;
            auto sp = item.first;
            auto ep = item.second;
            edges.push_back(std::get<0>(sp));
            edges.push_back(std::get<1>(sp));
            edges.push_back(std::get<2>(sp));
            edges.push_back(ep[0]);
            edges.push_back(ep[1]);
            edges.push_back(ep[2]);
            count += 1;
        }

        cnpy::npy_save(file_name, &edges[0],{(unsigned int)1, (unsigned int)count, (unsigned int)6},"w");
    }

    void RRTStar3D::save_obstacle(std::vector<SearchSpace::Rect> obstacles, std::string file_name){
        std::vector<float> obstacles_pose; 
        int count = 0;
        for(auto const& rect : obstacles){
                obstacles_pose.push_back(rect.min[0]);
                obstacles_pose.push_back(rect.min[1]);
                obstacles_pose.push_back(rect.min[2]);
                obstacles_pose.push_back(rect.max[0]);
                obstacles_pose.push_back(rect.max[1]);
                obstacles_pose.push_back(rect.max[2]);
                count += 1;
        }
        cnpy::npy_save(file_name, &obstacles_pose[0],{(unsigned int)1, (unsigned int)count, (unsigned int)6},"w");
    }

    void RRTStar3D::save_poses(Eigen::VectorXf start, Eigen::VectorXf end, std::string file_name){
       std::vector<float> obstacles_pose(6); 
       obstacles_pose[0] = start[0];
       obstacles_pose[1] = start[1];
       obstacles_pose[2] = start[2];
       obstacles_pose[3] = end[0];
       obstacles_pose[4] = end[1];
       obstacles_pose[5] = end[2];
       cnpy::npy_save(file_name, &obstacles_pose[0], {obstacles_pose.size()}, "w");
    }

    void RRTStar3D::save_path(std::vector<Eigen::VectorXf> path, std::string file_name){
       std::vector<float> projected_path;
        BOOST_LOG_TRIVIAL(info) << FCYN("RRTStar3D::save_path trajectory size: ") << path.size();
       for(auto const& way_point : path){
           projected_path.push_back(way_point[0]);
           projected_path.push_back(way_point[1]);
           projected_path.push_back(way_point[2]);
       }
       cnpy::npy_save(file_name, &projected_path[0], {path.size(), 3}, "w");
    }
}
}