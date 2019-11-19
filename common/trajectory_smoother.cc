#include "trajectory_smoother.h"

namespace kamaz {
namespace hagen {
            void TrajectorySmoother::set_smoother(std::string name){
                if(name == "bspline"){
                    _curve = new BSpline();
                }else if(name == "catmull"){
                   _curve = new CatmullRom();
                }
            }

            void TrajectorySmoother::clear_smoother(){
                _curve->clear();
            }

            void TrajectorySmoother::set_waypoints(std::vector<Eigen::Vector3d> waypoints, int _number_of_steps){
                 number_of_steps = _number_of_steps;
                _curve->clear();
                // std::cout<<"=====================31========================"<< std::endl;

                _curve->set_steps(waypoints.size()+number_of_steps);
                // std::cout<<"=====================32========================"<< std::endl;

                _curve->add_way_point(Vector(waypoints[0][0], waypoints[0][1], waypoints[0][2]));
                // std::cout<<"=====================33========================"<< std::endl;
                // std::cout<< waypoints.size() << std::endl;
                // int count =0;
                for(auto const way_point : waypoints){
                    // std::cout<< count << std::endl;
                    _curve->add_way_point(Vector(way_point[0], way_point[1], way_point[2]));
                    // count++;
                }
                // std::cout<<"=====================34========================"<< std::endl;

                _curve->add_way_point(Vector(waypoints.back()[0], waypoints.back()[1], waypoints.back()[2]));
                
                std::cout << "Nodes: " << _curve->node_count() << std::endl;
	            std::cout << "Total length: " << _curve->total_length() << std::endl;
            }

                
            std::vector<Eigen::Vector3d> TrajectorySmoother::get_smoothed_trajectory(){
                std::vector<Eigen::Vector3d> new_path;
                for (int i = 0; i < _curve->node_count(); ++i) {
                    Eigen::Vector3d pose(4);
                    auto node = _curve->node(i);
                    pose<< node.x, node.y, node.z, 1.0; 
                    new_path.push_back(pose);
	            }
                return new_path;
            }

            // https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d
            double TrajectorySmoother::get_distance(Eigen::Vector3d pont_a, Eigen::Vector3d pont_b, Eigen::Vector3d pont_c){
                Eigen::Vector3d d = (pont_c - pont_a);
                if(d.norm() != 0){
                    d = (pont_c - pont_a)/(pont_c - pont_a).norm();
                }
                Eigen::Vector3d v = (pont_b - pont_a);
                double t = v.dot(d);
                Eigen::Vector3d p = pont_a + t*d;
                double dis = (p-pont_b).norm();
                return dis;
            } 
    }
}
