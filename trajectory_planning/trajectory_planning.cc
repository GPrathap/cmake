#include "trajectory_planning.h"

namespace kamaz {
namespace hagen {

    TrajectoryPlanning::TrajectoryPlanning(float speed){
        max_speed = speed;
    }

    bool TrajectoryPlanning::generate_ts(std::vector<Eigen::VectorXf> path){
        int size_of_the_path = path.size();
        if(path.size()<1){
            return false;
        } 
        int vector_dim = path[0].size();
        way_points = Eigen::MatrixXf::Zero(size_of_the_path, vector_dim); 
        int row_index = 0;
        for(auto way_point : path){
            way_points.row(row_index) = way_point;
            row_index++;
        }
        Eigen::MatrixXf dis = (way_points.block(1, 0, size_of_the_path-1, vector_dim).array() 
                - way_points.block(0, 0, size_of_the_path-1, vector_dim).array()).pow(2).rowwise().sum().sqrt();
        float path_len = dis.sum();
        total_time = path_len/max_speed;
        Eigen::MatrixXf path_seg_length = dis.array().sqrt();
        for(int i=1; i<size_of_the_path-1; i++){
          path_seg_length(i, 0)= path_seg_length(i-1, 0) + path_seg_length(i, 0);
        }
        path_seg_length = path_seg_length.array()/path_seg_length(size_of_the_path-2, 0);
        
        time_segs.push_back(0.0);
        for(int i=1; i<size_of_the_path; i++){
          time_segs.push_back(path_seg_length(i-1, 0)*total_time);
        }
        return true;
    }


    void TrajectoryPlanning::get_desired_state(int qn, float time, std::vector<Eigen::VectorXf> states){
        
        if(time >= total_time){
            Eigen::MatrixXf point  = way_points.block(way_points.rows()-1, 0, 1, 3);
            Eigen::VectorXf pos = Eigen::Map<Eigen::RowVectorXf>(point.data(), 3);
            states.push_back(pos);
            Eigen::VectorXf vec = Eigen::VectorXf::Zero(3);
            Eigen::VectorXf acc = Eigen::VectorXf::Zero(3);
            states.push_back(vec);
            states.push_back(acc);
            return;
        }

        int k = closest(time).second;

        Eigen::MatrixXf pose_coeff(1, 8);
        pose_coeff << std::pow(time, 7.0f)
            ,std::pow(time, 6.0f)
            ,std::pow(time, 5.0f)
            ,std::pow(time, 4.0f)
            ,std::pow(time, 3.0f)
            ,std::pow(time, 2.0f)
            ,time
            ,1.0;
        
        Eigen::MatrixXf velocity_coeff(1, 8);
        velocity_coeff << 7.0*std::pow(time, 6.0f)
                , 6.0*std::pow(time, 5.0f)
                , 5.0* std::pow(time, 4.0f)
                , 4.0* std::pow(time, 3.0f)
                , 3.0* std::pow(time, 2.0f)
                , 2.0* std::pow(time, 1.0f)
                ,1.0
                ,0.0;
        
        Eigen::MatrixXf acceleration_coeff(1, 8);
        acceleration_coeff << 42.0*std::pow(time, 5.0f)
                    , 30.0*std::pow(time, 4.0f)
                    , 20.0* std::pow(time, 3.0f)
                    , 12.0* std::pow(time, 2.0f)
                    , 6.0* std::pow(time, 1.0f)
                    , 2.0
                    , 0.0
                    , 0.0;

        Eigen::MatrixXf position = pose_coeff*X.block(8*k, 0, 8, 3);
        Eigen::MatrixXf velocity = velocity_coeff*X.block(8*k, 0, 8, 3);
        Eigen::MatrixXf acceleration = acceleration_coeff*X.block(8*k, 0, 8, 3);

        Eigen::VectorXf pos = Eigen::Map<Eigen::RowVectorXf>(position.data(), 3);
        Eigen::VectorXf vel = Eigen::Map<Eigen::RowVectorXf>(velocity.data(), 3);
        Eigen::VectorXf acc = Eigen::Map<Eigen::RowVectorXf>(acceleration.data(), 3);

        states.push_back(pos);
        states.push_back(vel);
        states.push_back(acc);
        // std::cout << " k " << k << std::endl;

        // std::cout << "pose_coeff: "<< pose_coeff << std::endl;
        // std::cout << "velocity_coeff: "<< velocity_coeff << std::endl;
        // std::cout << "acceleration_coeff: "<< acceleration_coeff << std::endl;

        // std::cout << "position: "<< pos.transpose() << std::endl;
        // std::cout << "velocity: "<< vel.transpose() << std::endl;
        // std::cout << "acceleration: "<< acc.transpose() << std::endl;
    }

    std::pair<float, int > TrajectoryPlanning::closest(float value) {
        std::pair<float, int > result;
        int index = 0;
        for(auto point: time_segs){
            std::cout << point << std::endl;
            if(point > value){
                break;
            }
            index++;
        }
        if (index == (int)time_segs.size()) { 
            result.first = -1;
		    result.second = -1;
            return result; 
        }
        result.first = time_segs[index];
		result.second = index > 1 ? index -1 : 0;
        return result;
    }

    void TrajectoryPlanning::traj_opt7(){
        int m = way_points.rows();
        int n = way_points.cols();
        m = m - 1;
        int x_max = 8*m;
        X = Eigen::MatrixXf::Zero(x_max, n);
        A = Eigen::MatrixXf::Zero(n, x_max*x_max);
        Y = Eigen::MatrixXf::Zero(x_max, n);
        for(int i=0; i<n; i++){
            for(int b=0; b<x_max; b++){
                A(i, b*x_max+b) = 1*2.2204e-16;
            }
            int idx = 0;
            Eigen::MatrixXf coeff(1, 8);
            int colum_count = 0;
            int row_index = -1;
            for(int k = 0; k < m-1; k++){
                coeff << std::pow(time_segs[k+1], 7.0f)
                    ,std::pow(time_segs[k+1], 6.0f)
                    ,std::pow(time_segs[k+1], 5.0f)
                    ,std::pow(time_segs[k+1], 4.0f)
                    ,std::pow(time_segs[k+1], 3.0f)
                    ,std::pow(time_segs[k+1], 2.0f)
                    ,time_segs[k+1]
                    ,1.0;

                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                Y(idx, i) = way_points(k+1, i);
                idx = idx + 1;
                row_index++;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                Y(idx, i) = way_points(k+1, i);
                idx = idx + 1;
            }

            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 7.0*std::pow(time_segs[k+1], 6.0f)
                    , 6.0*std::pow(time_segs[k+1], 5.0f)
                    , 5.0* std::pow(time_segs[k+1], 4.0f)
                    , 4.0* std::pow(time_segs[k+1], 3.0f)
                    , 3.0* std::pow(time_segs[k+1], 2.0f)
                    , 2.0* std::pow(time_segs[k+1], 1.0f)
                    ,1.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 42.0*std::pow(time_segs[k+1], 5.0f)
                    , 30.0*std::pow(time_segs[k+1], 4.0f)
                    , 20.0* std::pow(time_segs[k+1], 3.0f)
                    , 12.0* std::pow(time_segs[k+1], 2.0f)
                    , 6.0* std::pow(time_segs[k+1], 1.0f)
                    , 2.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 210.0*std::pow(time_segs[k+1], 4.0f)
                    , 120.0*std::pow(time_segs[k+1], 3.0f)
                    , 60.0* std::pow(time_segs[k+1], 2.0f)
                    , 24.0* std::pow(time_segs[k+1], 1.0f)
                    , 6.0
                    , 0.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 840.0*std::pow(time_segs[k+1], 3.0f)
                    , 360.0*std::pow(time_segs[k+1], 2.0f)
                    , 120.0* std::pow(time_segs[k+1], 1.0f)
                    , 24.0
                    , 0.0
                    , 0.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 2520.0*std::pow(time_segs[k+1], 2.0f)
                    , 720.0*std::pow(time_segs[k+1], 1.0f)
                    , 120.0
                    , 0.0
                    , 0.0
                    , 0.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
            }
            colum_count = 0;
            for(int k = 0; k < m-1; k++){
                coeff << 5040.0*time_segs[k+1]
                    , 720.0
                    , 0.0
                    , 0.0
                    , 0.0
                    , 0.0
                    ,0.0
                    ,0.0;
                row_index++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
                colum_count++;
                A.block(i, x_max*row_index + 8*colum_count, 1, 8) = -1.0*coeff;
                Y(idx, i) = 0.0;
                idx = idx + 1;
            }
            int k = 0;
            colum_count = 0;
            coeff << std::pow(time_segs[k], 7.0f)
                    ,std::pow(time_segs[k], 6.0f)
                    ,std::pow(time_segs[k], 5.0f)
                    ,std::pow(time_segs[k], 4.0f)
                    ,std::pow(time_segs[k], 3.0f)
                    ,std::pow(time_segs[k], 2.0f)
                    ,time_segs[k]
                    ,1.0;
            row_index++;
            A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
            Y(idx, i) = way_points(k, i);
            idx = idx + 1;
            coeff << 7.0*std::pow(time_segs[k], 6.0f)
                    , 6.0*std::pow(time_segs[k], 5.0f)
                    , 5.0* std::pow(time_segs[k], 4.0f)
                    , 4.0* std::pow(time_segs[k], 3.0f)
                    , 3.0* std::pow(time_segs[k], 2.0f)
                    , 2.0* std::pow(time_segs[k], 1.0f)
                    ,1.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;
            coeff << 42.0*std::pow(time_segs[k], 5.0f)
                    , 30.0*std::pow(time_segs[k], 4.0f)
                    , 20.0* std::pow(time_segs[k], 3.0f)
                    , 12.0* std::pow(time_segs[k], 2.0f)
                    , 6.0* std::pow(time_segs[k], 1.0f)
                    , 2.0
                    ,0.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;
            coeff << 210.0*std::pow(time_segs[k], 4.0f)
                    , 120.0*std::pow(time_segs[k], 3.0f)
                    , 60.0* std::pow(time_segs[k], 2.0f)
                    , 24.0* std::pow(time_segs[k], 1.0f)
                    , 6.0
                    , 0.0
                    ,0.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + 8*colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;

            k = m-1;
            colum_count = 8*k;
            coeff << std::pow(time_segs[k+1], 7.0f)
                    ,std::pow(time_segs[k+1], 6.0f)
                    ,std::pow(time_segs[k+1], 5.0f)
                    ,std::pow(time_segs[k+1], 4.0f)
                    ,std::pow(time_segs[k+1], 3.0f)
                    ,std::pow(time_segs[k+1], 2.0f)
                    ,time_segs[k+1]
                    ,1.0;
            row_index++;
            A.block(i, x_max*row_index + colum_count, 1, 8) = coeff;
            Y(idx, i) = way_points(k+1, i);
            idx = idx + 1;

            coeff << 7.0*std::pow(time_segs[k+1], 6.0f)
                    , 6.0*std::pow(time_segs[k+1], 5.0f)
                    , 5.0* std::pow(time_segs[k+1], 4.0f)
                    , 4.0* std::pow(time_segs[k+1], 3.0f)
                    , 3.0* std::pow(time_segs[k+1], 2.0f)
                    , 2.0* std::pow(time_segs[k+1], 1.0f)
                    ,1.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;

            coeff << 42.0*std::pow(time_segs[k+1], 5.0f)
                    , 30.0*std::pow(time_segs[k+1], 4.0f)
                    , 20.0* std::pow(time_segs[k+1], 3.0f)
                    , 12.0* std::pow(time_segs[k+1], 2.0f)
                    , 6.0* std::pow(time_segs[k+1], 1.0f)
                    , 2.0
                    ,0.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;

            coeff << 210.0*std::pow(time_segs[k+1], 4.0f)
                    , 120.0*std::pow(time_segs[k+1], 3.0f)
                    , 60.0* std::pow(time_segs[k+1], 2.0f)
                    , 24.0* std::pow(time_segs[k+1], 1.0f)
                    , 6.0
                    , 0.0
                    ,0.0
                    ,0.0;
            row_index++;
            A.block(i, x_max*row_index + colum_count, 1, 8) = coeff;
            Y(idx, i) = 0.0;
            idx = idx + 1;
            
            Eigen::MatrixXf x_flat_mat = A.block(i, 0, 1, x_max*x_max);
            Eigen::MatrixXf y_flat_mat = Y.block(0, i, x_max, 1);
            Eigen::Map<Eigen::MatrixXf> x_flat_mat_map(x_flat_mat.data(), x_max, x_max);
            Eigen::MatrixXf x_flat_mat_map_new = x_flat_mat_map.transpose();
            Eigen::MatrixXf x_flat_mat_map_inv = x_flat_mat_map_new.inverse();
            X.block(0, i, x_max, 1) = x_flat_mat_map_inv*y_flat_mat;

        } 
    }



}
}