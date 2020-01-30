#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <cstring>
#include <fstream>
#include <cnpy.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef PATH_PLANNER_ELLIPSOID_KINORRTSTAR_H_
#define PATH_PLANNER_ELLIPSOID_KINORRTSTAR_H_

namespace kamaz {
namespace hagen {
        class Ellipsoid{
            public:
                Ellipsoid() = default;
                ~Ellipsoid() = default;
                double *ellipsoid_grid(int n, int ng );
                int ellipsoid_grid_count(int n, Eigen::Vector3d radios,
                                 Eigen::Vector3d center);
                int i4_ceiling (double x);
                void r83vec_print_part(int n, double a[], Eigen::Vector3d center_pose, Eigen::Matrix3d rotation_matrix, std::string file_name);
                void r8mat_write(std::string output_filename, int m, int n, double table[] );
                double r8vec_min(int n, double r8vec[]);
                void generate_points( int n, Eigen::Vector3d radios, Eigen::Vector3d center_pose
                    , Eigen::Matrix3d rotation_matrix, std::shared_ptr<Eigen::MatrixXd>& random_points_tank);
                void timestamp();
            private:
                double r[3];
                double c[3];
        };
    }
}
#endif

