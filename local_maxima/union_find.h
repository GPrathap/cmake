#ifndef SRC_UNIO_FIND_H_
#define SRC_UNIO_FIND_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <cassert>
#include <tuple>



namespace kamaz {
    namespace hagen{

class UnionFindDS{
 public:
    UnionFindDS() = default;
    ~UnionFindDS();
    
    void add(std::tuple<int, int, float> object, int weight);
    bool is_contains(std::tuple<int, int, float> object);
    std::tuple<int, int, float> get_items(std::tuple<int, int, float> object);
    void union_f(std::vector<std::tuple<int, int, float>> objects);
    bool is_equals_tuple(std::tuple<int, int, float> t1, std::tuple<int, int, float> t2);
    void print_tuple(std::tuple<int, int, float> object);
    void print_map_parent();
    void print_map();
    void print_vector( std::vector<std::tuple<int, int, float>> path);
    
   private: 
    std::map<std::tuple<int, int, float>, int> weights;
    std::map<std::tuple<int, int, float>, std::tuple<int, int, float>> parent;
    

};
    }
}

#endif 