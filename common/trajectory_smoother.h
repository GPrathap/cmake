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
                void set_waypoints(std::vector<Eigen::Vector3d> waypoints, int number_of_steps);
                std::vector<Eigen::Vector3d> get_smoothed_trajectory();
                void clear_smoother();
                double get_distance(Eigen::Vector3d pont_a, Eigen::Vector3d pont_b, Eigen::Vector3d pont_c);

            private:
                Curve* _curve;
                int number_of_steps;
        };
    }
}
#endif