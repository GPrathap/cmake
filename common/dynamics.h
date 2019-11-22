#ifndef _UTILS_DYNAMICS_H_
#define _UTILS_DYNAMICS_H_

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
#include <cnpy.h>
#include <random>
#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include <unsupported/Eigen/MatrixFunctions>

namespace kamaz {
  namespace hagen{
    class Dynamics {
      public:
        Dynamics();
        // Dynamics() = default;
        ~Dynamics() = default;
        Eigen::Matrix3d skewSymmetric(Eigen::Vector3d vector);
        Eigen::MatrixXd g(const Eigen::MatrixXd x, const Eigen::MatrixXd u);
        Eigen::MatrixXd f(const Eigen::MatrixXd x, const Eigen::MatrixXd u);
        size_t ell;
        // SymmetricMatrix<X_DIM> Q;
        double rotCost;
        // Matrix<X_DIM> xGoal, xStart;
        // SymmetricMatrix<U_DIM> R;
        
        double obstacleFactor;
        double scaleFactor;

        // Matrix<DIM> bottomLeft;
        // Matrix<DIM> topRight;
        double robotRadius;
        double dt;

        double gravity;         // gravity,   m/s^2
        // Matrix<3> eX, eY, eZ;   // unit vectors

        // quadrotor constants
        double mass;            // mass, kg
        Eigen::Matrix3d inertia;    // moment of inertia matrix 
        double momentConst;     // ratio between force and moment of rotor
        double dragConst;       // ratio between speed and opposite drag force
        double length;          // distance between center and rotor, m
        double minForce;        // minimum force of rotor, N
        double maxForce;        // maximum force of rotor, N
        Eigen::Vector3d eX;
        Eigen::Vector3d eY;
        Eigen::Vector3d eZ;

        // derivative constants
        Eigen::MatrixXd invInertia; // inverse of intertia matrix
        Eigen::MatrixXd uNominal;
        
      };
  }  
}
#endif  // _UTILS_DYNAMICS_H_
