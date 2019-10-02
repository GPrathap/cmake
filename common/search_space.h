#ifndef PATH_PLANNER_SEARCH_SPACE_H
#define PATH_PLANNER_SEARCH_SPACE_H

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
#include <Eigen/Dense>
#include <chrono> 
#include <vector>
#include <cnpy.h>
#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include "../utils/common_utils.h"
#include <random>

namespace kamaz {
namespace hagen {
        namespace bg = boost::geometry;
        namespace bgi = boost::geometry::index;
        class SearchSpace {
            typedef bg::model::point<float, 3, bg::cs::cartesian> point_t;
            typedef bg::model::box<point_t> box_t;
            typedef std::pair<box_t, uint64_t> value_t;
            typedef boost::geometry::box_view<box_t> box_view;
            typedef bgi::rtree<value_t, bgi::quadratic<8, 4>> RTree;

            public:
                SearchSpace();
                ~SearchSpace() = default;

                std::vector<std::uniform_real_distribution<>> uni_dis_vector;

                struct Rect {
                    Rect()  {}
                    Rect(float a_minX, float a_minY, float a_minZ, float a_maxX
                    , float a_maxY, float a_maxZ){
                        min[0] = a_minX;
                        min[1] = a_minY;
                        min[2] = a_minZ;

                        max[0] = a_maxX;
                        max[1] = a_maxY;
                        max[2] = a_maxZ;
                    }
                    float min[3];
                    float max[3];
                };

                struct Random_call
                {
                    Random_call(unsigned seed, int number_of_rand_points)
                    : _mt19937_64(seed)
                    , _uniform_int(0, number_of_rand_points)
                    {}

                    operator int() {
                        return _uniform_int(_mt19937_64);
                    }

                    std::mt19937 _mt19937_64;
                    std::uniform_int_distribution<int> _uniform_int;
                };

                void init_search_space(Eigen::VectorXf dimension_lengths
                        , int number_of_rand_points, float cube_size, float avoidance_width
                        , int number_of_tries_at_time, float voxel_side_length);
                void generate_random_objects(int num_of_objects);
                void insert_obstacles(std::vector<Rect> obstacles);
                void insert_trajectory(std::vector<Rect> trajectory);
                void search_all_obstacles();
                bool obstacle_free(Rect search_rect);
                bool obstacle_free(Eigen::VectorXf search_rect);
                Eigen::VectorXf sample_free();
                Eigen::VectorXf sample();
                std::vector<float> linspace(float start_in, float end_in, float step_size);
                bool collision_free(Eigen::VectorXf start, Eigen::VectorXf end, int r);
                void insert_obstacle(Eigen::VectorXf index);
                std::vector<Eigen::VectorXf> nearest_obstacles(Eigen::VectorXf x
                                    , int max_neighbours);
                std::vector<Eigen::VectorXf> 
                        nearest_point_on_trajectory(Eigen::VectorXf x
                        , int max_neighbours);
                
                void generate_samples_from_ellipsoid(Eigen::MatrixXf covmat, Eigen::Matrix3f rotation_mat
                , Eigen::VectorXf cent);


                void generate_search_sapce(Eigen::MatrixXf covmat, Eigen::Matrix3f rotation_mat
                        , Eigen::VectorXf cent, int npts);
                std::vector<float> arange(float start, float stop, float step);
                void save_samples(int index);
                void save_search_space(int index);
                void update_obstacles_map(std::vector<Rect> way_points);
                std::vector<Eigen::VectorXf> nearest_obstacles_to_current_pose(Eigen::VectorXf x
                                , int max_neighbours);
                float get_free_space(Eigen::VectorXf pose, std::vector<Eigen::VectorXf>& obs_poses
                                                    , int num_of_obs);
                float get_free_space(Eigen::VectorXf pose);
                
                int dementions = 3;
                Eigen::VectorXf dim_lengths;
                std::vector<uint64_t> res;
                std::shared_ptr<Eigen::MatrixXf> random_points_tank;
                float cube_length = 2;
                float avoidance_width = 1.5;
                int number_of_rand_points;
                Random_call* random_call;
                bool use_whole_search_sapce = false;
                float voxel_side_length = 0.1f;

                struct GeometryRTreeSearchCallback
                {
                    GeometryRTreeSearchCallback(SearchSpace* search_space): parent(search_space){

                    }
                    template <typename Value> void operator()(Value const& v)
                    {
                        // std::cout<< v.frist << std::endl;
                        // std::cout<< v.second << std::endl;
                        parent->res.push_back(v.second);
                    }
                    SearchSpace* parent;
                };

                GeometryRTreeSearchCallback geometry_rtree_callback;
                RTree bg_tree;
                RTree current_trajectory;
                RTree obs_tree;
                std::vector<Rect> random_objects;
                CommonUtils common_utils;
                int number_of_max_attempts;
                int number_of_points_in_random_tank;
                bool is_random_tank_is_ready = false;
                int obstacle_counter = 0;
        };
    }
}
#endif