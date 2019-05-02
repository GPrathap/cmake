#ifndef SEARCH_SPACE_H
#define SEARCH_SPACE_H

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
#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>

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
            // typedef bgi::rtree< value_t, bgi::linear<32, 8> > RTree;
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

                void init(Eigen::VectorXd dimension_lengths);
                void generate_random_objects(int num_of_objects);
                void insert_obstacles(std::vector<Rect> obstacles);
                void search_all_obstacles();
                bool obstacle_free(Rect search_rect);
                bool obstacle_free(Eigen::VectorXd search_rect);
                Eigen::VectorXd sample_free();
                Eigen::VectorXd sample();
                // bool comp(float a, float b);
                std::vector<float> linspace(float start_in, float end_in, float step_size);
                bool collision_free(Eigen::VectorXd start, Eigen::VectorXd end, int r);
                void insert(Eigen::VectorXd index);
                std::vector<Eigen::VectorXd> nearest(Eigen::VectorXd x, int max_neighbours);
   

                int dementions = 3;
                Eigen::VectorXd dim_lengths;
                std::vector<uint64_t> res;
                
                // std::default_random_engine generator_on_y;
                // std::default_random_engine generator_on_z;
                int cube_length = 2;
                

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

                // randomize rectangles
                std::vector<Rect> random_objects;
        };
    }
}
#endif