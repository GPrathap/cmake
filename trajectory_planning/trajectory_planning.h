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
#include <random>
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

namespace kamaz {
namespace hagen {
        class TrajectoryPlanning {
            public:
                TrajectoryPlanning(float speed = 2);
                ~TrajectoryPlanning() = default;

                bool generate_ts(std::vector<Eigen::VectorXf> path);
                void traj_opt7();
                void save_status(std::vector<std::vector<Eigen::VectorXf>>
                        , std::string file_name);

                void get_desired_state(float time
                        , std::vector<Eigen::VectorXf>& states);
                std::pair<float, int > closest(float value);

                void generate_target_trajectory(std::vector<Eigen::VectorXf>&  target_trajectory
  , std::string trajectory_to_be_flown_file_name);

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