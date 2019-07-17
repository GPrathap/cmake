#ifndef PATH_PLANNER_TRAJECTORY_SMOOTHER_H
#define PATH_PLANNER_TRAJECTORY_SMOOTHER_H

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
#include <Eigen/Dense>
#include <chrono> 
#include <vector>
#include "../spline/BSpline.h"
#include "../spline/CatmullRom.h"

#include <random>

namespace kamaz {
namespace hagen {
        class TrajectorySmoother {
            public:
                TrajectorySmoother() = default;
                ~TrajectorySmoother() = default;
                void set_smoother(std::string name);
                void set_waypoints(std::vector<Eigen::VectorXf> waypoints, int number_of_steps);
                std::vector<Eigen::VectorXf> get_smoothed_trajectory();
                void clear_smoother();
                float get_distance(Eigen::VectorXf pont_a, Eigen::VectorXf pont_b, Eigen::VectorXf pont_c);

            private:
                Curve* _curve;
                int number_of_steps;
        };
    }
}
#endif