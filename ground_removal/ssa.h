#ifndef GROUND_REMOVAL_SSA_H_
#define GROUND_REMOVAL_SSA_H_

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

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <Eigen/Eigenvalues> 
#include <complex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <parallel/algorithm>

namespace kamaz {
namespace hagen {
        class SingularSpectrumAnalysis {
            public:
                SingularSpectrumAnalysis(Eigen::Vector3d input_signal, int win_size);
                ~SingularSpectrumAnalysis() = default; 
                Eigen::Vector3d execute(int number_of_components, bool is_normalized);
                void save_data(int i);
                void save_vec(Eigen::Vector3d vec, std::string name);

            private:
                void normalize();
                void calculate_trajectory_matrix();
                Eigen::Vector3d  cross_corelation(Eigen::Vector3d x, Eigen::Vector3d y, int max_lags);
                Eigen::Vector3d conv(Eigen::Vector3d f, Eigen::Vector3d g);
                void calculate_covariance_matrix_toeplitz();
                Eigen::MatrixXd get_toeplitz_matrix(Eigen::Vector3d c, Eigen::Vector3d r);
                void calculate_eigne_vectors_and_values();
                void calculate_principle_components();
                void reconstruct_matrix();
                void save_mat(Eigen::MatrixXd matrix, std::string name);

                Eigen::Vector3d get_reconstructed_signal(int number_comps);
                std::vector<std::tuple<double, Eigen::Vector3d>> eigen_vectors_and_values; 
                Eigen::Vector3d feature_vector;
                Eigen::MatrixXd covariance_matrix;
                Eigen::MatrixXd trajectory_matrix;
                Eigen::MatrixXd principal_components;
                Eigen::MatrixXd reconstructed_matrix;
                Eigen::MatrixXd eigen_vectors;
                int M;
                int N;
                int number_of_lags;
            };
    }
}
#endif