#include "union_find.h"

namespace kamaz {
    namespace hagen {
    UnionFindDS::~UnionFindDS(){
        delete this;
    }

    void UnionFindDS::add(std::tuple<int, int, float> object, int weight){
            if(parent.find(object) != parent.end()){
                std::cout<< "Same object already exists" <<  std::get<0>(object) << " , "<< std::get<1>(object) << "we "<< weight <<std::endl;
            }else{
                parent[object] = object;
                weights[object] = weight;
            }
    }

    bool UnionFindDS::is_contains(std::tuple<int, int, float> object){
        if(parent.find(object) == parent.end()){
            return false;
        }else{
            return true;
        }
    }

    void UnionFindDS::print_tuple(std::tuple<int, int, float> object){
        std::cout<< "( " << std::get<0>(object) << " , " << std::get<1>(object) << ")" << std::endl;
    }

    void UnionFindDS::print_map(){
        std::cout<<"-----print map--------"<<std::endl;
        for(auto const& x : weights){
                    std::cout<< "( " << std::get<0>(x.first) << " , " << std::get<1>(x.first) << ") -> " << x.second << std::endl;
        }
        std::cout<<"-----end map--------"<<std::endl;
    }

    void UnionFindDS::print_vector( std::vector<std::tuple<int, int, float>> path){
        for(auto const& x : path){
               print_tuple(x); 
        }
    }

    void UnionFindDS::print_map_parent(){
        std::cout<<"-----print parent map--------"<<std::endl;
        // for(auto const& x : parent){
        //             std::cout<< "( " << std::get<0>(x.first) << " , " << std::get<1>(x.first) << ") -> " << "( " << std::get<0>(x.second) << " , " << std::get<1>(x.second)  << std::endl;
                   
        // }
        std::cout<<"-----end parent map--------"<<std::endl;
    }

    // Find and return the name of the set containing the object
    std::tuple<int, int, float> UnionFindDS::get_items(std::tuple<int, int, float> object){
        std::vector<std::tuple<int, int, float>> path;
        path.push_back(object);
        auto root = parent[object];
        // find path of objects leading to the root
        while(!is_equals_tuple(root, path.back())) {
            path.push_back(root);
            root = parent[object];
        }
        // compress the path and return
        for(auto const& ancestor: path){
            parent[ancestor] = root;
        }
        return root;
    }

    bool UnionFindDS::is_equals_tuple(std::tuple<int, int, float> t1, std::tuple<int, int, float> t2){
        if( (std::get<0>(t1) == std::get<0>(t2)) && (std::get<1>(t1) == std::get<1>(t2)) ){
            return true;
        }
        return false;
    } 

    // Find the sets containing the objects and merge them all
    void UnionFindDS::union_f(std::vector<std::tuple<int, int, float>> objects){
        std::vector<std::tuple<int, int, float>> roots;
        for(auto const& x: objects){
                roots.push_back(get_items(x));
        }
        int max_item = -500000;
        std::tuple<int, int, float> heaviest;
        for(auto const& r: roots){
            std::tuple<int, int, float> test_(std::get<0>(r), std::get<1>(r), std::get<2>(r));
            int _weight = weights[test_];
            if(_weight > max_item){
                max_item = _weight;
                heaviest = r;
            }
        }
        for(auto const& r: roots){
            if(!((std::get<0>(r) == std::get<0>(heaviest)) && (std::get<1>(r) == std::get<1>(heaviest)))){
                parent[r] = heaviest;
            }
        }
    }
}
}