#ifndef TRAJECTORY_PLANNING
#define TRAJECTORY_PLANNING

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <set>
#include <cstdlib>
#include <climits>
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
#include <fstream>


namespace kamaz {
namespace hagen {
        class TrajectoryPlanning {
            public:
                TrajectoryPlanning(float speed = 2);
                ~TrajectoryPlanning() = default;

                bool generate_ts(std::vector<Eigen::VectorXf> path);
                void traj_opt7();
                void get_desired_state(int qn, float time, std::vector<Eigen::VectorXf> states);
                std::pair<float, int > closest(float value);

                std::vector<float> time_segs;

                Eigen::MatrixXf way_points;
                float total_time;
                float max_speed;
                Eigen::MatrixXf X;
                Eigen::MatrixXf A;
                Eigen::MatrixXf Y;

            private:
            };
    }
}
#endif