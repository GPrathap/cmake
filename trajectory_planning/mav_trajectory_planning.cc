#include "mav_trajectory_planning.h"


namespace kamaz {
namespace hagen {

    MavTrajectoryPlanning::MavTrajectoryPlanning(double speed){
        max_speed = speed;
        is_set = false;
    }

    bool MavTrajectoryPlanning::generate_ts(std::vector<Eigen::VectorXd> path){
        int size_of_the_path = path.size();
        if(path.size()<1){
            return false;
        } 
//         std::cout<< "======segment_times==============>>>" << std::endl;
//         vertices.clear();
//         // derivative_to_optimize =
//          std::cout<< "1-----------------" << std::endl;
//         mav_trajectory_generation::Vertex start(dimension), middle1(dimension), end(dimension);
//   std::cout<< "12-----------------" << std::endl;
//         auto start_pos = Eigen::Vector4d(path[0][0], path[0][1], path[0][2], 1.0);
//         start.makeStartOrEnd(start_pos, derivative_to_optimize);
        // vertices.push_back(start);
        // auto previous_pos = start_pos;
        //   std::cout<< "1-3----------------" << std::endl;
        // for(int a = 1; a < path.size()-1; a++)
        // {
        //     auto current_pos = Eigen::Vector4d(path[a][0], path[a][1], path[a][2], 1.0);
        //     if((current_pos-previous_pos).norm()> 0.02){
        //         middle1.addConstraint(mav_trajectory_generation::derivative_order::POSITION, current_pos);
        //         vertices.push_back(middle1);
        //         previous_pos = current_pos;
        //     }
        // }

        mav_trajectory_generation::Vertex::Vector vertices1;
        const int dimension1 = 3;
        const int derivative_to_optimize1 = mav_trajectory_generation::derivative_order::SNAP;
        mav_trajectory_generation::Vertex start(dimension1), middle(dimension1), end(dimension1);
 
        Eigen::Vector3d v(0,0,1);
       
            start.makeStartOrEnd(v, derivative_to_optimize);
        // vertices.push_back(start);
        
        std::cout<< "14-----------------" << std::endl;
    
        // auto back_pose = path.back();
        // end.makeStartOrEnd(Eigen::Vector4d(back_pose[0], back_pose[1], back_pose[2], 1.0)
        //                 , derivative_to_optimize);
                          std::cout<< "15-----------------" << std::endl;
        // vertices.push_back(end);
          std::cout<< "16-----------------" << std::endl;
        // segment_times = estimateSegmentTimes(vertices, v_max, a_max);
          std::cout<< "17-----------------" << std::endl;
        return false;
    }

    void MavTrajectoryPlanning::save_status(std::vector<std::vector<Eigen::VectorXd>> status
    ,  std::string file_name){
       std::vector<double> quad_status; 
       for(auto sector: status){
            // std::cout<< status.size() << std::endl;
            for(auto val : sector){
                quad_status.push_back(val[0]);
                quad_status.push_back(val[1]);
                quad_status.push_back(val[2]);
            }
       }
       cnpy::npy_save(file_name, &quad_status[0], {quad_status.size()}, "w");
    }

    void MavTrajectoryPlanning::save_trajectory(std::vector<Eigen::VectorXd> trajectory_of_drone
    ,  std::string file_name){
       std::vector<double> quad_status; 
       for(auto sector: trajectory_of_drone){
            // std::cout<< status.size() << std::endl;
            quad_status.push_back(sector[0]);
            quad_status.push_back(sector[1]);
            quad_status.push_back(sector[2]);
       }
       cnpy::npy_save(file_name, &quad_status[0], {quad_status.size()}, "w");
    }


    void MavTrajectoryPlanning::get_desired_state(double current_sample_time_
                                                            , std::vector<Eigen::VectorXd>& states){
        if (current_sample_time_ <= trajectory_.getMaxTime()) {
            trajectory_msgs::MultiDOFJointTrajectory msg;
            mav_msgs::EigenTrajectoryPoint flat_state;
            bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
                trajectory_, current_sample_time_, &flat_state);
            if (!success) {
                std::cout<< "Error while generating trajectory of the path..."<< std::endl;
             //TODO do sothing for this condition...
            }else{
                mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state, &msg);
                Eigen::VectorXd pose = Eigen::VectorXd::Zero(3);
                Eigen::VectorXd vec = Eigen::VectorXd::Zero(3);
                Eigen::VectorXd acc = Eigen::VectorXd::Zero(3);
                auto position = msg.points[0].transforms[0].translation;
                auto velocity = msg.points[0].velocities[0].linear;
                auto acceleration = msg.points[0].accelerations[0].linear;
                pose << position.x, position.y, position.z;
                vec << velocity.x, velocity.y, velocity.z;
                acc << acceleration.x, acceleration.y, acceleration.z;
                states.push_back(pose);
                states.push_back(vec);
                states.push_back(acc);
            }
        } else {
            // Eigen::MatrixXd point  = way_points.block(way_points.rows()-1, 0, 1, 3);
            // Eigen::VectorXd pos = Eigen::Map<Eigen::RowVectorXd>(point.data(), 3);
            // states.push_back(pos);
            // Eigen::VectorXd vec = Eigen::VectorXd::Zero(3);
            // Eigen::VectorXd acc = Eigen::VectorXd::Zero(3);
            // states.push_back(vec);
            // states.push_back(acc);
            // TODO fix it 
            return;
        }
    }

    std::pair<double, int > MavTrajectoryPlanning::closest(double value) {
        std::pair<double, int > result;
        return result;
    }

    void MavTrajectoryPlanning::generate_target_trajectory(std::vector<Eigen::VectorXd>&  target_trajectory
                            , std::string trajectory_to_be_flown_file_name){
        return;
    }

    void MavTrajectoryPlanning::traj_opt7(){

        std::cout<< "======segment_times==============>>>" << segment_times.size() << std::endl;
        mav_trajectory_generation::PolynomialOptimization<10> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        mav_planning_msgs::PolynomialTrajectory4D segments_message;
        trajectoryToPolynomialTrajectoryMsg(trajectory, &segments_message);

        if (segments_message.segments.empty()) {
            ROS_WARN("Trajectory sampler: received empty waypoint message");
            return;
        } else {
            ROS_INFO("Trajectory sampler: received %lu waypoints",
                    segments_message.segments.size());
        }
        trajectory_.clear();
        bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
            segments_message, &trajectory_);
        if (!success) {
            return;
        }
        total_time = trajectory_.getMaxTime();
        is_set = true;
    }
    
}
}