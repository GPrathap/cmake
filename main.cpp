#include "ssa/ssa.h"

#include "./local_maxima/local_maxima_filter.h"
#include "./local_maxima/bilateral_filter.h"

#include <iostream>
#include <string>

#include <iostream>
#include <string>
#include <thread>
#include <mutex>


#include "./rrt_star/rrt_star_3d.h"
#include "./a_star/astar_improved.h"
#include "./apf/differential_equation_solver.h"
#include "./spline/Curve.h"
#include "./spline/BSpline.h"
#include "./spline/CatmullRom.h"
#include "./common/search_space.h"

using kamaz::hagen::SearchSpace;
using kamaz::hagen::AStarImproved;
using kamaz::hagen::APFCalculator;
using kamaz::hagen::DifferentialEquationSolver;

int main(){
  SearchSpace search_space;
  Eigen::MatrixXf covmat(3,3);
  Eigen::VectorXf center(3);
  center << 3, 3, 10;
  covmat(0,0) = 700;
  covmat(1,1) = 4;
  covmat(2,2) = 4;

  double roll, pitch, yaw;
  roll=0;
  pitch=M_PI/4;
  yaw=0;
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
  Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
  Eigen::Matrix3f rotation_matrix = q.matrix().cast<float>();
  search_space.generate_samples_from_ellipsoid(covmat, rotation_matrix, center, 1000);
  search_space.save_samples(1);
  return 0;
}

// int main(int argc, char** argv){
    
//     // cv::Mat I = cv::imread("/dataset/images/result/06/14391933_transformed.jpg", 0);
//     // if (I.empty())
//     // {
//     //     std::cout << "!!! Failed imread(): image not found" << std::endl;
//     // }

//     cv::Mat m1 = cv::Mat::zeros(10, 10, cv::DataType<float>::type);  
//     std::ifstream file("/dataset/test.txt");
//     float a  = 0;
//     for (int i = 0; i < 10; ++i) {
//         for (int j = 0; j < 10; ++j) {
//             file >> a;
//             m1.at<float>(i,j) = a; 
//         }
//     }

//     double min,  max;
//     cv::minMaxLoc(m1, &min, &max);
//     cv::Mat depth_normal = (m1/max)*255;
//     depth_normal.convertTo(depth_normal, CV_8U); 
//     std::cout <<  m1 << std::endl;
//     kamaz::hagen::LocalMaximaFilter local_maximum_filter;
//     cv::Mat filtered_map = Mat::zeros(10, 10, cv::DataType<float>::type);
//     int h = 10;
//     int w = 10; 
//     std::vector<std::tuple<int, int>> indices;
//     for(int i=0; i<h; i++){
//         for(int j=0; j<w; j++){
//             std::tuple<int, int> index_(i, j);
//             indices.push_back(index_);
//         }
//     }

//     std::vector<std::tuple<std::tuple<int, int, float>, kamaz::hagen::groups>> local_minima;
 
//     //local_maximum_filter.persistence(m1, local_minima);
    
//     // local_maximum_filter.persistence(m1, filtered_map);
//     local_maximum_filter.persistence_and_save_data(m1, filtered_map, 0);

    // std::cout <<  filtered_map << std::endl;


    // double min,  max;
    // cv::minMaxLoc(filtered_map, &min, &max);
    // cv::Mat depth_normal = (filtered_map/max);
    // std::cout <<  depth_normal << std::endl;
    // double min,  max;
    // cv::minMaxLoc(filtered_map, &min, &max);
    // cv::Mat depth_normal = (filtered_map/max)*255;
    // depth_normal.convertTo(depth_normal, CV_8U); 
    // cv::imwrite( "/dataset/images/result/03/"+ std::to_string(0) +"_depth_imageaterred_two_side_try_on_0.jpg", depth_normal);

    // // for(int i=0; i<50; i++){
    // //     std::cout<< "processing " << i << " th image;"<< std::endl;
        
    // //     cv::Mat image;
    // //     image = cv::imread("/dataset/images/"+ std::to_string(i) +"_depth_image.jpg", 0);
    //     image.convertTo(image, cv::DataType<float>::type, 1/255.0);

    //     std::vector<std::tuple<std::tuple<int, int>, kamaz::groups>> local_minima;
    //     // local_maximum_filter.persistence(image, local_minima);
    //     // cv::Mat filtered_map = cv::Mat::zeros( image.size(), CV_8UC1);
    //     cv::Mat filtered_map = cv::Mat::zeros(image.size(), cv::DataType<float>::type);
    //     int h = image.rows;
    //     int w = image.cols; 
    //     std::vector<std::tuple<int, int>> indices;
    //     for(int i=0; i<h; i++){
    //         for(int j=0; j<w; j++){
    //             std::tuple<int, int> index_(i, j);
    //             indices.push_back(index_);
    //         }
    //     }
    //     kamaz::LocalMaximaFilter local_maximum_filter;
    //     local_maximum_filter.persistence(image, filtered_map, indices);
    //     //local_maximum_filter.~LocalMaximaFilter();
    //     cv::imwrite( "/dataset/images/result/02/"+ std::to_string(i) +"_depth_image_filtered_bilateral.png", filtered_map);
        
    //     // double min,  max;
    //     // cv::minMaxLoc(filtered_map, &min, &max);
    //     // cv::Mat depth_normal = (filtered_map/max)*255;
    //     // depth_normal.convertTo(depth_normal, CV_8U);

    //     cv::resize(filtered_map, filtered_map, cv::Size(), 1.0, 2);
    //     cv::resize(image, image, cv::Size(),1.0, 2); 

    //     cv::Mat prossed_image = (filtered_map + image)/2;

    //     cv::imwrite( "/dataset/images/result/02/"+ std::to_string(i) +"_depth_image_filtered_image.png", prossed_image);

    // }

    // kamaz::BilateralFilter bilateral_filter;
    // cv::Mat src;
    // src = cv::imread( "/dataset/images/0_depth_image.jpg", 0 );
    // imwrite("original_image_grayscale.png", src);

    // if ( !src.data )
    // {
    //     printf("No image data \n");
    //     return -1;
    // }

    // cv::Mat filteredImageOwn = bilateral_filter.bilateralFilterOwn(src, 5, 12.0, 16.0);
    // cv::imwrite("/dataset/images/0_filtered_image_own.png", filteredImageOwn);

    // kamaz::APFCalculator apfCalculator;
    // apfCalculator.epsilonGoal = 0.05;
    // std::vector<Mat> obstacles;

    // apfCalculator.initPositions((Mat)(Mat_<double>(1, 3) << -9.5, 1.5, -1.5),
    //                             (Mat)(Mat_<double>(1, 3) << 8.0, 0.0, 0.0));

    // obstacles.push_back((Mat)(Mat_<double>(1, 4) << 0.0, -1.0, 0.0, 1.0));
    // obstacles.push_back((Mat) (Mat_<double>(1, 4) << 4.0, 0.0, 1.0, 2.0));
    // obstacles.push_back((Mat) (Mat_<double>(1, 4) << -3.0, 1.0, -1.0, 2.0));
    // apfCalculator.initArena(obstacles);

    // std::cout << "Stating point = " << apfCalculator.qStart  << std::endl;
    // std::cout << "Ending point = " << apfCalculator.qGoal << std::endl;

    // kamaz::DifferentialEquationSolver differential_solver;
    // differential_solver.init(apfCalculator);

    // std::vector<tuple<double, double, double>> projected_trajectory;
    // projected_trajectory.clear();

    // //creating a thread to execute our task
	// std::thread worker([&]()
	// {
	// 	differential_solver.run(projected_trajectory);
	// });
    // // send data to the worker thread
    // {
    //     std::lock_guard<std::mutex> lk(differentiator_lock);
    //     differentiator_ready = true;
    // }
    // differentiator_cv.notify_one();
    // // wait for the worker
    // {
    //     std::unique_lock<std::mutex> lk(differentiator_lock);
    //     differentiator_cv.wait(lk, []{return differentiator_processed;});
    // }

    // std::cout << "Back in main()'\n' " << projected_trajectory.size()<< std::endl;
    // worker.join();


    // kamaz::hagen::Hagen hagen;
    // int message = 90;
    //  //creating a thread to execute our task
    // std::cout << "---------1--------------- "<< std::endl;
	
    // std::cout << "---------2-------------- "<< std::endl;
    
    // std::thread point_clod_thread = hagen.onPointCloudThread(message);
    // std::thread object_detection_thread = hagen.detectObjectThread(std::ref(on_object_detection)
    //                                             , std::ref(condition_on_object_detection));

    // std::thread estimae_trajectory_thread = hagen.estimateTrajectoryThread(on_trajectory_estimation
    //                               , condition_on_trajectory_estimation);
    // std::thread try_to_avoid_obstacles = hagen.tryToEscapeObstaclesThread(std::ref(on_object_detection),
    //                               std::ref(on_trajectory_estimation),
    //                               std::ref(condition_on_object_detection),
    //                               std::ref(condition_on_trajectory_estimation));
    
    // // std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    // std::cout << "main process 4000 ms" << " ms\n";
    // hagen.do_processing = false;

    // {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    //     std::cout << "main process 4000 ms" << " ms\n";
    //     std::lock_guard<std::mutex> lk(on_object_detection);
    //     hagen.do_processing = true;

    // }
    // condition_on_object_detection.notify_one();

    // // wait for the worker
    // {
    //     std::unique_lock<std::mutex> lk(differentiator_lock);
    //     differentiator_cv.wait(lk, []{return differentiator_processed;});
    // }

    // std::cout << "Back in main()'\n' "<< std::endl;
    // hagen.processing_point_cloud = false;
    // hagen.processing_object_detection = false;
    // point_clod_thread.join();
    // object_detection_thread.join();
    // estimae_trajectory_thread.join();
    //    try_to_avoid_obstacles.join();

//     return 0;
// }



















// int main(){

//   Eigen::VectorXf a(8);
//   a << 1, 2, 3, 4, 5, 6, 7, 8;
//   Eigen::VectorXf b(8);
//   b << 12,13,14,15,16,17,18,19;

//   // Eigen::MatrixXf m(4,3);
//   // m <<  1, 2, 3,
//   //       4, 5, 6, 
//   //       7, 8,9,
//   //       10,11,12;
  
  
//   //   std::cout << m.block(0,0,m.rows(),2).rowwise().sum() << std::endl;
  

//   // kamaz::hagen::SingularSpectrumAnalysis ssa(a, 2);
//   // auto f = ssa.execute(2, true);
//   // std::cout<< f.transpose() << std::endl;

//   cv::Mat image;
//   image = cv::imread("/dataset/images/result/8/15207_angle_image.jpg", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
  

  
//   // Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic> bb;
//   // cv::cv2eigen(image, bb); 
//   // for(auto i(0); i< bb.rows(); i++){
//   //   kamaz::hagen::SingularSpectrumAnalysis ssa(bb.row(i), 16);
//   //   auto f = ssa.execute(3, true);
//   //   ssa.save_data(i);
//   //   std::cout<< f.transpose() << std::endl;
//   //   ssa.save_vec(f, "smoothed_signal_"+ std::to_string(i));
//   // }

//   // Eigen::MatrixXf SingularSpectrumAnalysis::get_smoothed_image(cv::Mat& image){
//         Eigen::MatrixXf smoothed_image(image.rows, image.cols);
//         Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> eigen_image;
//         cv::cv2eigen(image, eigen_image); 
//         for(auto i(0); i< eigen_image.rows(); i++){
//             kamaz::hagen::SingularSpectrumAnalysis ssa(16, 3, true);
//             ssa.init(eigen_image.row(i));
//             smoothed_image.row(i) = ssa.execute();
//         }
// }




































































// int main()
// {
//     kamaz::hagen::RRTStar3D rrtstart3d;
//     Eigen::VectorXd x_dimentions(3);
//     x_dimentions << 100, 100, 100;
//     auto map_dim = rrtstart3d.get_search_space_dim(x_dimentions);
//     // auto obstacles = rrtstart3d.get_obstacles();
//     auto obstacles = rrtstart3d.get_random_obstacles(10, x_dimentions);
//     // std::cout<< "-----1" << std::endl;
//     Eigen::VectorXd x_init(3);
//     x_init << 0, 0, 0 ;
//     Eigen::VectorXd x_goal(3);
//     x_goal << 99, 99, 99;

//     std::vector<Eigen::VectorXd> Q;
//     Eigen::VectorXd dim_in(2);
//     dim_in << 8, 4;
//     Q.push_back(dim_in);
//     // std::cout<< "-----1" << std::endl;
//     int r = 1;
//     int max_samples = 10000;
//     int rewrite_count = 32;
//     float proc = 0.1;

//     kamaz::hagen::SearchSpace X;
//     X.init(map_dim);
//     X.insert_obstacles(obstacles);

//     rrtstart3d.rrt_init(Q, max_samples, r, proc, rewrite_count);
//     auto path = rrtstart3d.rrt_planner_and_save(X, x_init, x_goal);
   

//     Curve* bspline_curve = new BSpline();
//     Curve* catmulll_curve = new CatmullRom();

// 	  bspline_curve->set_steps(100);
//     catmulll_curve->set_steps(100);

//     bspline_curve->add_way_point(Vector(path[0][0], path[0][1], path[0][2]));
//     catmulll_curve->add_way_point(Vector(path[0][0], path[0][1], path[0][2]));

//     for(auto const way_point : path){
//       std::cout<<"main: "<< way_point.transpose() << std::endl;
//       bspline_curve->add_way_point(Vector(way_point[0], way_point[1], way_point[2]));
//       catmulll_curve->add_way_point(Vector(way_point[0], way_point[1], way_point[2]));
//     }

//     bspline_curve->add_way_point(Vector(path.back()[0], path.back()[1], path.back()[2]));
//     catmulll_curve->add_way_point(Vector(path.back()[0], path.back()[1], path.back()[2]));

//     std::cout << "nodes: " << bspline_curve->node_count() << std::endl;
// 	  std::cout << "total length: " << bspline_curve->total_length() << std::endl;

//     std::vector<Eigen::VectorXd> new_path_bspline;
//     std::vector<Eigen::VectorXd> new_path_catmull;
//     // if(path.size()>0){
//     //   new_path_bspline.push_back(path[0]);
//     //   new_path_catmull.push_back(path[0]);
//     // }
//     for (int i = 0; i < bspline_curve->node_count(); ++i) {
// 		  Eigen::VectorXd pose(3);
//       auto node = bspline_curve->node(i);
//       pose<< node.x, node.y, node.z; 
//       new_path_bspline.push_back(pose);
// 	  }
//     for (int i = 0; i < catmulll_curve->node_count(); ++i) {
// 		  Eigen::VectorXd pose(3);
//       auto node = catmulll_curve->node(i);
//       pose<< node.x, node.y, node.z; 
//       new_path_catmull.push_back(pose);
// 	  }
//     // if(path.size()>0){
//     //   new_path_bspline.push_back(path.back());
//     //   new_path_catmull.push_back(path.back());
//     // }

//     rrtstart3d.save_path(new_path_bspline, "/dataset/rrt_path_modified.npy");
//     rrtstart3d.save_path(new_path_catmull, "/dataset/rrt_path_modified_catmull.npy");
// 	  delete bspline_curve;
//     // AStarImproved path_planner;
//     // auto projected_trajectory = path_planner.astar_planner_and_save(X, x_init, x_goal);

//     return 0;
// }
// // // Generic functor
// // template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
// // struct Functor
// // {
// //     typedef _Scalar Scalar;
// //     enum {
// //         InputsAtCompileTime = NX,
// //         ValuesAtCompileTime = NY
// //     };
// //     typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
// //     typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
// //     typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

// //     int m_inputs, m_values;

// //     Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
// //     Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

// //     int inputs() const { return m_inputs; }
// //     int values() const { return m_values; }

// // };

// // struct my_functor : Functor<float>
// // {
// //     my_functor(void): Functor<float>(2,2) {}
// //     int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
// //     {
// //         // Implement y = 10*(x0+3)^2 + (x1-5)^2
// //         fvec(0) = 10.0*pow(x(0)+3.0,2) +  pow(x(1)-5.0,2);
// //         fvec(1) = 0;

// //         return 0;
// //     }
// // };

// // int main(int argc, char *argv[]) {
// //     Eigen::VectorXd x(2);
// //     x(0) = 2.0;
// //     x(1) = 3.0;
// //     std::cout << "x: " << x << std::endl;

// //     my_functor functor;
// //     Eigen::NumericalDiff<my_functor> numDiff(functor);
// //     Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>,float> lm(numDiff);
// //     lm.parameters.maxfev = 2000;
// //     lm.parameters.xtol = 1.0e-10;
// //     std::cout << lm.parameters.maxfev << std::endl;

// //     int ret = lm.minimize(x);
// //     std::cout << lm.iter << std::endl;
// //     std::cout << ret << std::endl;

// //     std::cout << "x that minimizes the function: " << x << std::endl;

// //     std::cout << "press [ENTER] to continue " << std::endl;
// //     std::cin.get();
// //     return 0;
// // }