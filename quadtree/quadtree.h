#ifndef QUAD_TREE
#define QUAD_TREE

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
#include <cnpy.h>

#include <stack> 

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <Eigen/Eigenvalues> 
#include <complex>

namespace kamaz {
namespace hagen {
        class QuadTree {
            public:
               
                struct Point
                {
                    Point(float x, float y, float angle, float depth)
                    : x(x), y(y), angle(angle), depth(depth)
                    {}
                    float x;
                    float y;
                    float angle;
                    float depth;

                    float operator-(Point a){
                        return (a.x-x) + (a.y-y);
                    }
                    
                };

                struct Rect
                {
                    Rect(float x, float y, float w, float h):x(x), y(y), w(w), h(h){}
                    Rect() {}

                    float x;
                    float y;
                    float w;
                    float h;
                    
                    bool contains(Point point){
                        if(!((x <= point.x) && (point.x <= x + w))){
                            return false;
                        }
                        if(!((y <= point.y) && (point.y <= y + h))){
                            return false;
                        }
                        return true;
                    }

                    std::string debug_string(){
                        return " x: " + std::to_string(x) + " y: " + std::to_string(y) + " w: " + std::to_string(w) + " h: " + std::to_string(h);
                    }

                    std::vector<Rect> split(){
                        float w2 = w/2;
                        float h2 = h/2;
                        std::array<float, 2> a = {0, h2};
                        std::array<float, 2> b = {0, w2};
                        std::vector<std::array<float, 2>> bounds;
                        for(float i= 0; i<2; i++){
                            for (float j= 0; j < 2; j++)
                            {
                                std::array<float, 2> bound {a[i], b[j]};
                                bounds.push_back(bound);
                            }
                        }
                        std::vector<Rect> quadrants;
                        for(auto const bound : bounds){
                            auto rect = Rect(x+bound[1], y+bound[0], w2, h2);
                            quadrants.push_back(rect);
                        }
                        return quadrants;
                    }
                };

                struct Node
                {
                    std::vector<Point> val;
                    Rect bounds;
                    int level;

                    Node(std::vector<Point> val, Rect bounds, int level):val(val), bounds(bounds), level(level)
                    {
                        sons_names.push_back(0); // nw
                        sons_names.push_back(1); // ne
                        sons_names.push_back(2); // sw
                        sons_names.push_back(3); // se
                    }

                    Node(Node* father_, Rect bounds, int level):bounds(bounds), level(level)
                    {
                        father = father_;
                        sons_names.push_back(0); // nw
                        sons_names.push_back(1); // ne
                        sons_names.push_back(2); // sw
                        sons_names.push_back(3); // se
                    }

                    Node* father;

                    std::vector<Node> sons;
                    std::vector<int> sons_names;

                    std::vector<int> get_sons(){
                        if(sons.size()==0){
                            std::vector<int> dump;
                            return dump;
                        }
                        return sons_names;
                    }

                    bool leaf(){
                        for(auto const son : sons){
                            if(son.val.size()>0){
                                return false;
                            }
                        }
                        return true;
                    }

                    std::vector<Point> get_val(){
                        return val;
                    }

                    int get_val_size(){
                        return val.size();
                    }

                    int get_level(){
                        return level;
                    }

                    std::vector<Rect> bounds_split(){
                        return bounds.split();
                    }

                };

                QuadTree(std::vector<Point> data, float width, float height, float angle_threshold);
                ~QuadTree() = default; 
                void _split();
                void in_order_traversal(Node root);
                
                Rect rect;
                int size;
                Node root;
                float width;
                float height;
                float angle_threshold;

            private:
            
            };
    }
}
#endif