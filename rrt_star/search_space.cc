#include "search_space.h"

namespace kamaz {
namespace hagen {
    SearchSpace::SearchSpace(): geometry_rtree_callback(this) {
        
    }

    void SearchSpace::init(Eigen::VectorXd dimension_lengths){
        dim_lengths = dimension_lengths;
        std::uniform_real_distribution<> distribution_x(dimension_lengths[0], dimension_lengths[1]);
        std::uniform_real_distribution<> distribution_y(dimension_lengths[2], dimension_lengths[3]);
        std::uniform_real_distribution<> distribution_z(dimension_lengths[4], dimension_lengths[5]);
        uni_dis_vector.push_back(distribution_x);
        uni_dis_vector.push_back(distribution_y);
        uni_dis_vector.push_back(distribution_z);
        // random_objects = obstacles;
    }

    void SearchSpace::generate_random_objects(int num_of_objects){
        for (int i = 0 ; i < num_of_objects ; ++i)
        {
            float min_x = rand() % 1000;
            float min_y = rand() % 1000;
            float min_z = rand() % 1000;
            float w = 1 + rand() % 100;
            float h = 1 + rand() % 100;
            float d = 1 + rand() % 100;
            random_objects.push_back(Rect(min_x, min_y, min_z, min_x+w, min_y+h, min_z+d));
        }
    }

    void SearchSpace::insert_obstacles(){
        if(random_objects.size() == 0){
            std::cout<< "No obstacle to be inserted" << std::endl;
        }
        for(size_t i = 0; i < random_objects.size(); i++){
            Rect const& r = random_objects[i];
            box_t b(point_t(r.min[0], r.min[1], r.min[2]), point_t(r.max[0], r.max[1], r.max[2]));
            bg_tree.insert(value_t(b, i));
        }
    }

    void SearchSpace::insert(Eigen::VectorXd index){
        //TODO not sure this method
        box_t b(point_t(index[0], index[1], index[2])
        , point_t(index[0]+cube_length, index[1]+cube_length
        , index[2]+cube_length));
        bg_tree.insert(value_t(b, 0));
    }

    std::vector<Eigen::VectorXd> SearchSpace::nearest(Eigen::VectorXd x, int max_neighbours){
        std::vector<value_t> returned_values;
        // box_t pt(point_t(x[0], x[1], x[2]),
        // point_t(x[0] + cube_length, x[1] + cube_length, x[2] + cube_length));
        // bg_tree.query(bgi::nearest(point_t(x[0], x[1], x[2]), max_neighbours), std::back_inserter(returned_values));
    // std::cout<< "-----1" << std::endl;
        std::vector<Eigen::VectorXd> neighbour_points;
        for ( RTree::const_query_iterator it = bg_tree.qbegin(bgi::nearest(point_t(x[0], x[1], x[2]), max_neighbours)) ;
                it != bg_tree.qend() ; ++it )
        {
            Eigen::VectorXd pose(3);
            auto cube = (*it).first;

            // std::cout<< "SearchSpace::nearest:  distance: " << bg::distance(cube, point_t(x[0], x[1], x[2])) << std::endl;

            float min_x = bg::get<bg::min_corner, 0>(cube);
            float min_y = bg::get<bg::min_corner, 1>(cube);
            float min_z = bg::get<bg::min_corner, 2>(cube);

            float max_x = bg::get<bg::max_corner, 0>(cube);
            float max_y = bg::get<bg::max_corner, 1>(cube);
            float max_z = bg::get<bg::max_corner, 2>(cube);

            // std::cout<< "SearchSpace::nearest: "<< min_x << ","<<min_y << ", "<< min_z << ", " << max_x << ", "<< max_y << ", " << max_z << std::endl;

            // pose << (min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2;
            pose << min_x, min_y, min_z;
            neighbour_points.push_back(pose);
        
        }
        return neighbour_points;        
        
        

        // for(value_t const& v: returned_values){
        //     Eigen::VectorXd pose(3);
        //     auto cube = v.first;
        //     float min_x = bg::get<bg::min_corner, 0>(cube);
        //     float min_y = bg::get<bg::min_corner, 1>(cube);
        //     float min_z = bg::get<bg::min_corner, 2>(cube);

        //     float max_x = bg::get<bg::max_corner, 0>(cube);
        //     float max_y = bg::get<bg::max_corner, 1>(cube);
        //     float max_z = bg::get<bg::max_corner, 2>(cube);

        //     // pose << (min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2;
        //     pose << min_x, min_y, min_z;
        //     neighbour_points.push_back(pose);
        // }
        // return neighbour_points;
    }

    bool SearchSpace::obstacle_free(Rect search_rect){
        box_t search_box(
        point_t(search_rect.min[0], search_rect.min[1], search_rect.min[2]),
        point_t(search_rect.max[0], search_rect.max[1], search_rect.max[2]));
        size_t sum = 0;
        // boost::timer t;
        res.clear();
        sum += bg_tree.query(bgi::intersects(search_box)
        , boost::make_function_output_iterator(geometry_rtree_callback));
        // float s = t.elapsed();
        // std::cout << s << " " << sum << std::endl;
        return sum > 0 ? false : true;
    }

    bool SearchSpace::obstacle_free(Eigen::VectorXd search_rect){
        box_t search_box(
        point_t(search_rect[0], search_rect[1], search_rect[2]),
        point_t(search_rect[0]+cube_length, search_rect[1]+cube_length, search_rect[2]+cube_length));
        size_t sum = 0;
        // boost::timer t;
        res.clear();
        sum += bg_tree.query(bgi::intersects(search_box)
        , boost::make_function_output_iterator(geometry_rtree_callback));
        // float s = t.elapsed();
        // std::cout <<" SearchSpace::obstacle_free: sum: " << sum << std::endl;
        return sum > 0 ? false : true;
    }

    Eigen::VectorXd SearchSpace::sample(){
        Eigen::VectorXd random_pose(3);
        std::default_random_engine generator_on_x;
        generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
        auto x_on = uni_dis_vector[0](generator_on_x);
        generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
        auto y_on = uni_dis_vector[1](generator_on_x);
        generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
        auto z_on = uni_dis_vector[2](generator_on_x);
        random_pose<< x_on, y_on, z_on;
        return random_pose;
    }

    Eigen::VectorXd SearchSpace::sample_free(){
        while(true){
            auto x = sample();
            std::cout<< x << std::endl;
            if(obstacle_free(x)){
                return x;
            }
        }
    }

    bool SearchSpace::collision_free(Eigen::VectorXd start, Eigen::VectorXd end, int r){
        auto dist = (start - end).norm();
        float resolution = std::ceil(dist/r);
        std::vector<float> res_on_x = linspace(start[0], end[0], resolution);
        std::vector<float> res_on_y = linspace(start[1], end[1], resolution);
        std::vector<float> res_on_z = linspace(start[2], end[2], resolution);
        // std::cout<< "===================" << std::endl;
        // std::cout<<  res_on_x.size() << " " << res_on_y.size() <<" "<< res_on_z.size() << std::endl;
        int len = std::min({res_on_x.size(), res_on_y.size(), res_on_z.size()}
        , [](const int s1, const int s2) -> bool{
                return s1 < s2;
        });
        std::cout<< "SearchSpace::collision_free:: len: " << len << std::endl;
        for(int i=0; i<len; i++){
            Eigen::VectorXd search_rect(3);
            search_rect<< res_on_x[i], res_on_y[i], res_on_z[i];
            // std::cout<< search_rect.transpose() << std::endl;
            if(!obstacle_free(search_rect)){
                return false;
            }
        }
        return true;
    }

    // bool SearchSpace::comp(float a, float b){ 
    //     return (a < b); 
    // } 

    std::vector<float> SearchSpace::linspace(float start_in, float end_in, float step_size)
    {
        std::vector<float> linspaced;
        float start = static_cast<float>(start_in);
        float end = static_cast<float>(end_in);
        float num = static_cast<float>(step_size);
        if (num == 0) { return linspaced; }
        if (num == 1)
        {
            linspaced.push_back(start);
            return linspaced;
        }
        float delta = (end - start) / (num - 1);
        for(int i=0; i < num-1; ++i)
            {
            linspaced.push_back(start + delta * i);
            }
        linspaced.push_back(end);
        return linspaced;
    }

    void SearchSpace::search_all_obstacles(){
        Rect search_rect(4000, 4000, 6000, 6000, 6000, 6000);
        box_t search_box(
        point_t(search_rect.min[0], search_rect.min[1], search_rect.min[2]),
        point_t(search_rect.max[0], search_rect.max[1], search_rect.max[2]));
        size_t sum = 0;
        boost::timer t;
        res.clear();
        sum += bg_tree.query(bgi::intersects(search_box)
        , boost::make_function_output_iterator(geometry_rtree_callback));
        float s = t.elapsed();
        std::cout << s << " " << sum << std::endl;
    }
}
}


