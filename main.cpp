#include <iostream>
#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include "./rrt_star/rrt_star_3d.h"
#include "./a_star/astar_improved.h"
#include "./apf/differential_equation_solver.h"

using kamaz::hagen::SearchSpace;
using kamaz::hagen::AStarImproved;
using kamaz::hagen::APFCalculator;
using kamaz::hagen::DifferentialEquationSolver;

int main()
{
    

    kamaz::hagen::RRTStar3D rrtstart3d;
    Eigen::VectorXd x_dimentions(3);
    x_dimentions << 100, 100, 100;
    auto map_dim = rrtstart3d.get_search_space_dim(x_dimentions);
    // auto obstacles = rrtstart3d.get_obstacles();
    auto obstacles = rrtstart3d.get_random_obstacles(5, x_dimentions);
    // std::cout<< "-----1" << std::endl;
    Eigen::VectorXd x_init(3);
    x_init << 0, 0, 0 ;
    Eigen::VectorXd x_goal(3);
    x_goal << 99, 99, 99;

    std::vector<Eigen::VectorXd> Q;
    Eigen::VectorXd dim_in(2);
    dim_in << 8, 4;
    Q.push_back(dim_in);
    // std::cout<< "-----1" << std::endl;
    int r = 1;
    int max_samples = 10000;
    int rewrite_count = 32;
    float proc = 0.1;

    if(x_goal[0] >= w){
      x_goal[0] = w - 1;
    }
    if(x_goal[1] >= h ){
      x_goal[1] = h - 1;
    }
    if(x_goal[2] >= d){
      x_goal[2] = d - 1;
    }
    if(x_goal[0] < 0 ||x_goal[1] < 0 || x_goal[2] < 0){
      return 0;
    }

    kamaz::hagen::SearchSpace X;
    X.init(map_dim);
    X.insert_obstacles(obstacles);

    rrtstart3d.rrt_init(Q, max_samples, r, proc, rewrite_count);
    auto path = rrtstart3d.rrt_planner_and_save(X, x_init, x_goal);
   
    AStarImproved path_planner;
    auto projected_trajectory = path_planner.astar_planner_and_save(X, x_init, x_goal);

    return 0;
}










































































// // Generic functor
// template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
// struct Functor
// {
//     typedef _Scalar Scalar;
//     enum {
//         InputsAtCompileTime = NX,
//         ValuesAtCompileTime = NY
//     };
//     typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
//     typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
//     typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

//     int m_inputs, m_values;

//     Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
//     Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

//     int inputs() const { return m_inputs; }
//     int values() const { return m_values; }

// };

// struct my_functor : Functor<float>
// {
//     my_functor(void): Functor<float>(2,2) {}
//     int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
//     {
//         // Implement y = 10*(x0+3)^2 + (x1-5)^2
//         fvec(0) = 10.0*pow(x(0)+3.0,2) +  pow(x(1)-5.0,2);
//         fvec(1) = 0;

//         return 0;
//     }
// };

// int main(int argc, char *argv[]) {
//     Eigen::VectorXd x(2);
//     x(0) = 2.0;
//     x(1) = 3.0;
//     std::cout << "x: " << x << std::endl;

//     my_functor functor;
//     Eigen::NumericalDiff<my_functor> numDiff(functor);
//     Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>,float> lm(numDiff);
//     lm.parameters.maxfev = 2000;
//     lm.parameters.xtol = 1.0e-10;
//     std::cout << lm.parameters.maxfev << std::endl;

//     int ret = lm.minimize(x);
//     std::cout << lm.iter << std::endl;
//     std::cout << ret << std::endl;

//     std::cout << "x that minimizes the function: " << x << std::endl;

//     std::cout << "press [ENTER] to continue " << std::endl;
//     std::cin.get();
//     return 0;
// }