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
     // create the rectangle passed into the query
    // Eigen::VectorXd dimension_lengths(6);
    // dimension_lengths << 0, 512, 0, 512, 0, 512;
    // std::vector<Eigen::VectorXf> obstacles;
    // Eigen::VectorXd start(3);
    // Eigen::VectorXd end(3);
    // start<< 12, 45, 67;
    // end << 234, 234,456;

    // // SearchSpace search_space;
    // // search_space.init(dimension_lengths, obstacles);
    // // search_space.generate_random_objects(200);
    // // search_space.insert_obstacles();
    // // // search_space.search_all_obstacles();
    // // // search_space.sample_free();
    // // bool free_path = search_space.collision_free(start, end, 1);
    // // std::cout<< free_path << std::endl;

    kamaz::hagen::RRTStar3D rrtstart3d;
    // std::cout<< "-----1" << std::endl;
    Eigen::VectorXd x_dimentions(3);
    x_dimentions << 100, 100, 100;
    auto map_dim = rrtstart3d.get_search_space_dim(x_dimentions);
    // std::cout<< "-----1" << std::endl;

    // auto obstacles = rrtstart3d.get_obstacles();
    auto obstacles = rrtstart3d.get_random_obstacles(2, x_dimentions);
    // std::cout<< "-----1" << std::endl;
    Eigen::VectorXd x_init(3);
    x_init << -9.5, 0.01, 0.01 ;
    Eigen::VectorXd x_goal(3);
    x_goal << 8, 0, 0;

    std::vector<Eigen::VectorXd> Q;
    Eigen::VectorXd dim_in(2);
    dim_in << 8, 4;
    Q.push_back(dim_in);
    // std::cout<< "-----1" << std::endl;
    int r = 1;
    int max_samples = 10000;
    int rewrite_count = 32;
    float proc = 0.1;

    kamaz::hagen::SearchSpace X;
    X.init(map_dim);
    X.random_objects = obstacles;
    X.insert_obstacles();
    // // std::cout<< "-----1" << std::endl;
    // auto rrtstar = rrtstart3d.rrt_search(X, Q, x_init
    //                 , x_goal, max_samples, r, proc, rewrite_count);
    // // std::cout<< "-----1" << std::endl;
    // std::vector<Eigen::VectorXd> path;
    // path = rrtstar.rrt_star();

    // rrtstart3d.save_edges(rrtstar.trees);
    rrtstart3d.save_obstacle(obstacles);
    rrtstart3d.save_poses(x_init, x_goal);
    // rrtstart3d.save_path(path);

    APFCalculator apfCalculator;
    apfCalculator.epsilonGoal = 0.05;
    std::vector<cv::Mat> obstacles_previous_try = apfCalculator.get_obstacles(obstacles);
    apfCalculator.initArena(obstacles_previous_try);
    apfCalculator.initPositions((cv::Mat)(cv::Mat_<double>(1, 3) << x_init[0],x_init[1], x_init[2]),
                                    (cv::Mat)(cv::Mat_<double>(1, 3) << x_goal[0], x_goal[1], x_goal[2]));

    std::cout << "Stating point = " << apfCalculator.qStart  << std::endl;
    std::cout << "Ending point = " << apfCalculator.qGoal << std::endl;

    DifferentialEquationSolver differential_solver;
    differential_solver.init(apfCalculator);
    std::vector<tuple<double, double, double>> projected_trajectory;
    projected_trajectory.clear();
    bool is_estimated = differential_solver.run(projected_trajectory);

    // AStarImproved path_planner;
    // AStarImproved::Options options;
    // path_planner.init_planner(X, options);

    // int rows = x_dimentions[0];
    // int cols = x_dimentions[1];
    // int depth = x_dimentions[2];

    // AStarImproved::pose start;
    // start.x = x_init[0];
    // start.y = x_init[1];
    // start.z = x_init[2];

    // AStarImproved::pose end;
    // end.x =  x_goal[0];
    // end.y =  x_goal[1];
    // end.z =  x_goal[2];

    // if(end.x >= rows){
    //   end.x = rows - 1;
    // }
    // if(end.y >= cols ){
    //   end.y = cols - 1;
    // }
    // if(end.z >= depth){
    //   end.z = depth - 1;
    // }
    // if(end.x < 0 || end.y < 0 || end.z < 0){
    //   return false;
    // }
    // int start_idx = path_planner.to1D(start.x, start.y, start.z);
    // int goal_idx = path_planner.to1D(end.x, end.y, end.z);
    // std::vector<int> paths(rows*cols*depth);
    // bool success = path_planner.astar(start_idx, goal_idx, false, paths);
    // std::vector<Eigen::VectorXd> projected_trajectory;
    // if(success){
    //     std::cout<< "Path is found"<< std::endl;
    //     auto path_idx = goal_idx;
    //     int path_node_id=0;
    //     while(path_idx != start_idx){
    //         auto path_position = path_planner.to3D(path_idx);
    //         Eigen::VectorXd poseh(3);
    //         poseh << path_position[0], path_position[1], path_position[2];
    //         projected_trajectory.push_back(poseh);
    //         path_idx = paths[path_idx];
    //         path_node_id++;
    //     }
    //     auto start_position = path_planner.to3D(path_idx);
    //     Eigen::VectorXd poseh(3);
    //     poseh << start_position[0], start_position[1], start_position[2];
    //     projected_trajectory.push_back(poseh);
    // }else{
    //     std::cout<< "Path is not found"<< std::endl;
    // }
    // path_planner.save_path(projected_trajectory);

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