#include "dynamics.h"

namespace kamaz {
namespace hagen{


    Dynamics::Dynamics(){
        gravity = 9.80665; // gravity,   m/s^2
        eX = Eigen::Vector3d::UnitX();
        eY = Eigen::Vector3d::UnitY();
        eZ = Eigen::Vector3d::UnitZ();
        // Quadrotor parameters
        mass        = 0.5;             // mass, kg  (source: paper)
        inertia     = 0.05*Eigen::MatrixXd::Identity(3,3); // moment of inertia matrix 
        momentConst = 1.5e-9 / 6.11e-8;  // ratio between force and moment of rotor
        dragConst   = 0.15; //0.15;
        length      = 0.3429/2;          // distance between center and rotor, m
        invInertia   =  20*Eigen::MatrixXd::Identity(3,3);
        uNominal = Eigen::MatrixXd::Zero(4, 1);
        uNominal<< gravity*mass/4, gravity*mass/4, gravity*mass/4, gravity*mass/4;
        // Control parameters
        double ell = 150;
        double initdt = 0.05;
        // timeFactor = 3;
        // Q = 500*identity<X_DIM>();
        // Q(X_DIM-1,X_DIM-1) = 0;
        rotCost = 0.3; 
        obstacleFactor = 1;
        scaleFactor = 10;
        // R = 20*identity<U_DIM>();
        // Environment parameters
        robotRadius = length + 0.1;
        dt = 0.5;
    }

    Eigen::Matrix3d Dynamics::skewSymmetric(Eigen::Vector3d vector){
        Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();
        skew_mat << 0, -vector[2], vector[1], vector[2], 0, -vector[0], -vector[1], vector[0], 0;  
        return skew_mat;
    }  

    Eigen::MatrixXd Dynamics::g(const Eigen::MatrixXd x, const Eigen::MatrixXd u) {
        Eigen::MatrixXd k1 = f(x, u);
        auto fff = 0.5*dt*k1;
        Eigen::MatrixXd k2 = f(x + fff, u);
        Eigen::MatrixXd k3 = f(x + 0.5*dt*k2, u);
        Eigen::MatrixXd k4 = f(x + dt*k3, u);
        return x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
    }

    Eigen::MatrixXd Dynamics::f(const Eigen::MatrixXd x, const Eigen::MatrixXd u) {
        Eigen::MatrixXd xDot(x.rows(), x.cols());
        Eigen::MatrixXd p = x.block<3,1>(0,0);
        Eigen::MatrixXd v = x.block<3,1>(3,0);
        Eigen::MatrixXd r = x.block<3,1>(6,0);
        Eigen::MatrixXd w = x.block<3,1>(9,0);
        Eigen::Vector3d eX = Eigen::Vector3d::UnitX();
        Eigen::Vector3d eY = Eigen::Vector3d::UnitY();
        Eigen::Vector3d eZ = Eigen::Vector3d::UnitZ();

        // std::cout<< " p " << p << std::endl;
        // std::cout<< " v " << v << std::endl;
        // std::cout<< " r " << r << std::endl;
        // std::cout<< " w " << w << std::endl;
        // std::cout<< " skewSymmetric(r) " << skewSymmetric(r) << std::endl;
        // // \dot{p} = v
        // // xDot.insert(0, 0, v);
        xDot.block<3,1>(0,0) = v;
        // // \dot{v} = [0,0,-g]^T + R*exp([r])*[0,0,(f_1 + f_2 + f_3 + f_4) / m]^T; 
         
        xDot.block<3,1>(3,0) = (-gravity*eZ) + (skewSymmetric(r).exp())*((u.sum()/mass)*eZ) - dragConst*v/mass;
        // std::cout<<"====>" << fghd.cols() << " === "<< fghd.rows() << std::endl;
        // std::cout<< " v0 " <<  skewSymmetric(r).exp() << std::endl;
        // std::cout<< " v2 " << skewSymmetric(r) << std::endl;
        
        // // \dot{r} = w + 0.5*skewSymmetric(r)*w + (1.0/tr(~r*r))*(1.0 - 0.5*sqrt(tr(~r*r))/tan(0.5*sqrt(tr(~r*r))))*skewSymmetric(r)*(skewSymmetric(r)*w)
        double l = sqrt(r(0)*r(0)+r(1)*r(1)+r(2)*r(2));
        if (0.5*l > 0.0) {
            xDot.block<3,1>(6,0) = w + (0.5*skewSymmetric(r)*w) + (1.0 - 0.5*l/tan(0.5*l))*skewSymmetric(r / l)*(skewSymmetric(r / l)*w);
        } else {
            xDot.block<3,1>(6,0) = w;
        }
        // // \dot{w} = J^{-1}*([l*(f_2 - f_4), l*(f_3 - f_1), (f_1 - f_2 + f_3 - f_4)*k_M]^T - [w]*J*w)
        xDot.block<3,1>(9,0) = invInertia*( (length*(u(1) - u(3))*eX) + (length*(u(2) - u(0))*eY)
                    + ((u(0) - u(1) + u(2) - u(3))*momentConst*eZ) - (skewSymmetric(w)*inertia*w));
        return xDot;
    }

}
}