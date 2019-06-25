#include "quadtree.h"

namespace kamaz {
namespace hagen {

    QuadTree::QuadTree(std::vector<Point> data, float width, float height, float angle_threshold)
        :width(width), height(height), rect(0, 0, width, height), root(data, rect, 0){
            if(data.size()>0){
                _split();
            }
    }

    void QuadTree::_split(){
        std::deque<Node*> node_list;
        node_list.push_back(&root);
        while (!node_list.empty()){
            Node* node = node_list.front();
            node_list.pop_front();
            if(node->get_val_size() <= 1){
                continue;
            }
            if(node->leaf()){
                auto rects = node->bounds_split();
                Node nw(node, rects[0], node->get_level()+1);
                Node ne(node, rects[1], node->get_level()+1);
                Node sw(node, rects[2], node->get_level()+1);
                Node se(node, rects[3], node->get_level()+1);
                node->sons.push_back(nw);
                node->sons.push_back(ne);
                node->sons.push_back(sw);
                node->sons.push_back(se);
            }
            for(Point val : node->get_val()){
                // std::cout<< "===========================" << std::endl;
                for(int son_name : node->get_sons()){
                    // std::cout<< "son boundary" << node->sons[son_name].bounds.debug_string () << std::endl;
                    if(node->sons[son_name].bounds.contains(val)){
                        // std::cout<< "value:" << val.x << "," << val.y << std::endl;
                        node->sons[son_name].val.push_back(val);
                        break;
                    }  
                }
            }
            // node->val.clear();
            for(int son_name : node->get_sons()){
                node_list.push_back(&(node->sons[son_name]));
            }
            // std::cout<< "size of node list: "<< node_list.size() << std::endl;
        }
    }

    

    void QuadTree::in_order_traversal(Node root){
        std::stack<Node> s1;
        std::stack<Node> s2;
        s1.push(root);
        // std::cout<< "=============start======="<< std::endl;
        while(s1.size()>0){
            auto node = s1.top();
            s1.pop();
            s2.push(node);
            // std::cout<< "==========node.sons.size(): "<< node.sons.size()<< std::endl;
            if(node.sons.size()>0){
                for(int son_name : node.get_sons()){
                    s1.push(node.sons[son_name]);
                }
            }
        }
        // std::cout<< "s2 size: "<< s2.size()<<std::endl;

        while(s2.size()){
            auto node = s2.top();
            s2.pop();
            if(node.leaf() && node.val.size()==1){
                Point point =  node.val[0];
                // std::cout<< "=======endnode==========" << std::endl;
                // std::cout<<"x: " << point.x << " y: " 
                // << point.y << " depth: "<< point.depth << " angle: "<< point.angle<< std::endl;
                Node* father = node.father;
                // std::cout<< "=======father other son==========" << std::endl;

                for(int son_name : father->get_sons()){
                    auto son_val = father->sons[son_name].val;
                    // std::cout<< "=============son_val=======" << son_val.size()<< std::endl;
                    if(father->sons[son_name].val.size()>0){
                        for(Point son_point: son_val){
                            if(!(point-son_point) == 0){
                // std::cout<<"x: " << son_point.x << " y: " 
                // << son_point.y << " depth: "<< son_point.depth << " angle: "<< son_point.angle << " level:" <<father->sons[son_name].level << std::endl;
                                 if(std::abs(son_point.angle-point.angle)< angle_threshold){

                                }
                            }
                        }
                            
                    }
                }
            }
        }
    }
}
}