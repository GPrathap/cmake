#include "./local_maxima/local_maxima_filter.h"
// #include "./local_maxima/bilateral_filter.h"

#include <iostream>
#include <string>

#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include "./rrt_star/rrt_star_3d.h"
// #include "./a_star/astar_improved.h"
// #include "./apf/differential_equation_solver.h"
// #include "./spline/Curve.h"
#include "./spline/BSpline.h"
// #include "./spline/CatmullRom.h"
#include "./common/search_space.h"
#include "../ground_removal/ssa.h"
#include "quadtree/quadtree.h"
#include <random>
#include "trajectory_planning/trajectory_planning.h"
#include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "../include/pcl_types.h"
#include "../ground_removal/depth_ground_remover.h"
#include "../projections/projection_params.h"
#include "../utils/cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "spline/polynomial.h"
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

using kamaz::hagen::SearchSpace;
// using kamaz::hagen::AStarImproved;
// using kamaz::hagen::APFCalculator;
// using kamaz::hagen::DifferentialEquationSolver;
using kamaz::hagen::QuadTree;
using kamaz::hagen::TrajectoryPlanning;
using kamaz::hagen::LocalMaximaFilter;
using kamaz::hagen::SingularSpectrumAnalysis;
using kamaz::hagen::PCLPoint;
using kamaz::hagen::PointCloud;
using kamaz::hagen::PointCloudPtr;


// int main(){
//   std::vector<Eigen::VectorXd> path;
//   Eigen::VectorXd f(3);
//   f<<0, 5, 5;
//   path.push_back(f);
//   f<<2.7, 4.9, 3.75;
//   path.push_back(f);
//   f<<2.7, 4.9, 1.25;
//   path.push_back(f);
//   f<<5.7, 4.9, 1.75;
//   path.push_back(f);
//   f<<5.7, 4.9, 4.75;
//   path.push_back(f);
//   f<<8.7, 4.9, 4.25;
//   path.push_back(f);
//   f<<8.7, 4.9, 1.25;
//   path.push_back(f);
//   f<<11.7, 4.9, 1.25;
//   path.push_back(f);
//   f<<11.7, 4.9, 4.75;
//   path.push_back(f);
//   f<<14.7, 4.9, 4.25;
//   path.push_back(f);
//   f<<14.7, 4.9, 1.25;
//   path.push_back(f);
//   f<<17.7, 4.9, 1.75;
//   path.push_back(f);
//   f<<17.7, 4.9, 4.75;
//   path.push_back(f);
//   f<<20.0, 5,5;
//   path.push_back(f);
//   std::cout<< "size of the path: "<< path.size() << std::endl;

//   TrajectoryPlanning trajectory_planning;

// //   trajectory_planning.generate_target_trajectory(path, "/dataset/result/9/path1.csv");
//   int size_of_the_path = path.size();
//   trajectory_planning.generate_ts(path);

//   for (auto time_stamp : trajectory_planning.time_segs){
//     std::cout<< "---" << time_stamp << std::endl;
//   }

//   std::cout<< "Total time: " << trajectory_planning.total_time << std::endl;
//   trajectory_planning.traj_opt7();

//   float cstep = 0.05;
//   float time = 0.0;
//   float tstep = 0.01;

//   int max_iter = (int) (trajectory_planning.total_time / cstep); 
//   std::cout<< "===============max_iter============"<< max_iter << std::endl;
  
//   std::vector<std::vector<Eigen::VectorXd>> paths_points;
//   for (int iter =1; iter < max_iter; iter++){
//     std::vector<Eigen::VectorXd> desired_state;
//     trajectory_planning.get_desired_state(time+cstep, desired_state);
//     paths_points.push_back(desired_state);
//     std::cout<< "==>: "<<  desired_state[0] << std::endl;
//     time = time + cstep;
//   }
//   std::cout<< paths_points.size() << std::endl;
//   trajectory_planning.save_status(paths_points, "/dataset/result/9/trajectory_planning_pose.csv");

//   //   std::cout<< trajectory_planning.X << std::endl;
//   // //   std::cout<< trajectory_planning.time_segs << std::endl;
//   // Eigen::MatrixXf A = Eigen::MatrixXf::Zero(3, 6*6);
//   // int x_max = 6;
//   // Eigen::MatrixXf g = A.block<1, x_max*x_max>(2,0);
//   // for(int k=0; k<x_max; k++){
//   //   g(0, k*x_max+k) = 1;
//   // }
//   // Eigen::Map<Eigen::MatrixXf> M2(g.data(), x_max, x_max);

//   // std::cout<< M2 << std::endl; 
//   // Eigen::MatrixXf gg = g.resize( 8*14, 8*14);

//   return 0;
// }
// int main(){

  // auto point = QuadTree::Point(2, 4, 0.1, 3.4);
  // int width = 10;
  // int height = 20;
  // auto rect = QuadTree::Rect(0,0, width, height);
  // std::cout<< rect.contains(point)<< std::endl;
  // auto bounds = rect.split();
  // for(auto const rect: bounds){
  //   std::cout<< rect.x << " " << rect.y << " " << rect.w << " " << rect.h << std::endl;
  // }

  // std::random_device rd;     
  // std::mt19937 rng(rd());   
  // std::uniform_int_distribution<int> rand_x(0,width);
  // std::uniform_int_distribution<int> rand_y(0,height);
  // std::vector<QuadTree::Point> data;
  // for(int i=0; i<100; i++){
  //   auto x = rand_x(rng);
  //   auto y = rand_y(rng);
  //   auto point = QuadTree::Point(x, y, 0.1, 3.4);
  //   std::cout<< x << " " << y << std::endl;
  //   data.push_back(point);
  // }
  // std::cout<< "=====================" << std::endl;
  // QuadTree quadtree(data, width, height, 5);
  // quadtree.in_order_traversal(quadtree.root);

  // SearchSpace search_space;
  // Eigen::MatrixXf covmat(3,3);
  // Eigen::VectorXd center(3);
  // center << 0,0,0;
  // covmat(0,0) = 8;
  // covmat(1,1) = 4;
  // covmat(2,2) = 4;

  // Eigen::Vector3f a(1,0,0);
  // Eigen::Vector3f b(0,1,1);
  // Eigen::VectorXd fg(6);
  // search_space.init(fg);
  // // Random_call random_call(std::chrono::system_clock::now().time_since_epoch().count(), 100);
  // // int num = random_call;
  // // std::cout<<  num << std::endl;
  // int f = *(search_space.random_call);
  // std::cout<<  f << std::endl;
  // // Eigen::Matrix3f rotation_matrix = search_space.get_roration_matrix(a, b);
  // // search_space.generate_samples_from_ellipsoid(covmat, rotation_matrix, center, 1000);
  // // search_space.save_samples(1);

  // Eigen::Vector3f pont_a(3);
  // pont_a << 3,0,0;
  // Eigen::Vector3f pont_b(3);
  // pont_b << 2,5,5;
  // Eigen::Vector3f pont_c(3);  
  // pont_c << 2,0,0;
  // float dis = search_space.get_distance(pont_a, pont_b, pont_c);
  // // std::cout<< dis << std::endl;
//   return 0;
// }#include <iostream>
// #include <string>
// using namespace std;

//     void strrev(string *at_front) {
//         string *at_back = at_front;
//         char transfer_back;
//         int elem = 0;

//         while(at_back->operator[](elem) != '\0') {
//             elem++;
//         }

//         elem = elem;
//         for(int i = 0; i <elem; i++) {
//             transfer_back = at_front->operator[](i);
//             at_front->operator[](i) = at_back->operator[](elem);
//             at_back->operator[](elem) = transfer_back;
//             elem--;
//         }
//     }



// int main(int argc, char** argv){

    // string str = "ereh txet yna";
    // std::cout<< str << std::endl;
    // string *point_str = &str;

    // strrev(point_str);

    // cout << *point_str << endl;

    // SearchSpace search_space;
    // Eigen::Vector3f pont_a(3);
    // pont_a << 3,0,0;
    // Eigen::Vector3f pont_b(3);
    // pont_b << 3,0.4,0;
    // Eigen::Vector3f pont_c(3);  
    // pont_c << 3,0,0;
    // float dis = search_space.get_distance(pont_a, pont_b, pont_c);
    // std::cout<< dis << std::endl;
    
    // cv::Mat I = cv::imread("/dataset/images/result/06/14391933_transformed.jpg", 0);
    // if (I.empty())
    // {
    //     std::cout << "!!! Failed imread(): image not found" << std::endl;
    // }

    // cv::Mat m1 = cv::Mat::zeros(10, 10, cv::DataType<float>::type);  
    // std::ifstream file("/dataset/test.txt");
    // float a  = 0;
    // for (int i = 0; i < 10; ++i) {
    //     for (int j = 0; j < 10; ++j) {
    //         file >> a;
    //         m1.at<float>(i,j) = a; 
    //     }
    // }

    // double min, max;
    // cv::minMaxLoc(m1, &min, &max);
    // cv::Mat depth_normal = (m1/max)*255;
    // depth_normal.convertTo(depth_normal, CV_8U); 
    // std::cout <<  m1 << std::endl;
    // kamaz::hagen::LocalMaximaFilter local_maximum_filter;
    // cv::Mat filtered_map = cv::Mat::zeros(10, 10, cv::DataType<float>::type);
    // int h = 10;
    // int w = 10; 
    // std::vector<std::tuple<int, int>> indices;
    // for(int i=0; i<h; i++){
    //     for(int j=0; j<w; j++){
    //         std::tuple<int, int> index_(i, j);
    //         indices.push_back(index_);
    //     }
    // }

    // std::vector<std::tuple<std::tuple<int, int, float>, kamaz::hagen::groups>> local_minima;
    // local_maximum_filter.persistence_and_save_data(m1, filtered_map, 0);
    // local_maximum_filter.persistence(m1, filtered_map);

            // cv::imwrite( "/dataset/images/result/02/"+ std::to_string(i) +"_depth_image_filtered_image.png", prossed_image);


    // std::cout <<  filtered_map << std::endl;
    // cv::minMaxLoc(filtered_map, &min, &max);
    // cv::Mat depth_normal = (filtered_map/max);
    // std::cout <<  depth_normal << std::endl;
    // double min,  max;
    // cv::minMaxLoc(filtered_map, &min, &max);
    // cv::Mat depth_normal = (filtered_map/max)*255;
    // depth_normal.convertTo(depth_normal, CV_8U); 
    // cv::imwrite( "/dataset/images/result/03/"+ std::to_string(0) +"_depth_imageaterred_two_side_try_on_0.jpg", depth_normal);

    // for(int i=0; i<50; i++){
    //     std::cout<< "processing " << i << " th image;"<< std::endl;
        
    //     cv::Mat image;
    //     image = cv::imread("/dataset/images/"+ std::to_string(i) +"_depth_image.jpg", 0);
        // image.convertTo(image, cv::DataType<float>::type, 1/255.0);

        // std::vector<std::tuple<std::tuple<int, int>, kamaz::groups>> local_minima;
        // // local_maximum_filter.persistence(image, local_minima);
        // // cv::Mat filtered_map = cv::Mat::zeros( image.size(), CV_8UC1);
        // cv::Mat filtered_map = cv::Mat::zeros(image.size(), cv::DataType<float>::type);
        // int h = image.rows;
        // int w = image.cols; 
        // std::vector<std::tuple<int, int>> indices;
        // for(int i=0; i<h; i++){
        //     for(int j=0; j<w; j++){
        //         std::tuple<int, int> index_(i, j);
        //         indices.push_back(index_);
        //     }
        // }
        // kamaz::LocalMaximaFilter local_maximum_filter;
        // local_maximum_filter.persistence(image, filtered_map, indices);
        // //local_maximum_filter.~LocalMaximaFilter();
        // cv::imwrite( "/dataset/images/result/02/"+ std::to_string(i) +"_depth_image_filtered_bilateral.png", filtered_map);
        
        // // double min,  max;
        // // cv::minMaxLoc(filtered_map, &min, &max);
        // // cv::Mat depth_normal = (filtered_map/max)*255;
        // // depth_normal.convertTo(depth_normal, CV_8U);

        // cv::resize(filtered_map, filtered_map, cv::Size(), 1.0, 2);
        // cv::resize(image, image, cv::Size(),1.0, 2); 

        // cv::Mat prossed_image = (filtered_map + image)/2;

        // cv::imwrite( "/dataset/images/result/02/"+ std::to_string(i) +"_depth_image_filtered_image.png", prossed_image);

    //   return 0;
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
















// #include <opencv2/opencv.hpp>
// #include <iostream>
 
// using namespace std;
// using namespace cv;


// int main(){

//   Eigen::VectorXf a(4);
//   Eigen::VectorXf b(4);
//   a << 1,4,1,0;
//   b << 1,2,1,0;
//   Eigen::VectorXf projection_vector = b.head(3) - a.head(3);
//   Eigen::VectorXf projection_vector_z = projection_vector;
//   projection_vector_z[2] = 0.0;

//   float angle = projection_vector.dot(projection_vector_z)/(projection_vector.norm()*projection_vector_z.norm());
//   angle = std::acos(angle);
//   Eigen::VectorXf normalized_vector = projection_vector.normalized();
//   auto angle_z = std::atan2(normalized_vector[1], normalized_vector[0]);
//   std::cout<< "======================" << angle_z*(180/3.14) << std::endl;    
//   std::cout<< "======================" << angle*(180/3.14) << std::endl;     


  // // Eigen::VectorXd a(8);
  // a << 1, 2, 3, 4, 5, 6, 7, 8;
  // Eigen::VectorXd b(8);
  // b << 12,13,14,15,16,17,18,19;

  // Eigen::MatrixXf m(4,3);
  // m <<  1, 2, 3,
  //       4, 5, 6, 
  //       7, 8,9,
  //       10,11,12;
  
  
  //   std::cout << m.block(0,0,m.rows(),2).rowwise().sum() << std::endl;
  

  // kamaz::hagen::SingularSpectrumAnalysis ssa(a, 2);
  // auto f = ssa.execute(2, true);
  // std::cout<< f.transpose() << std::endl;

  // cv::Mat image;
  // image = cv::imread("/dataset/images/result/10/3008_angle_img.jpg", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
  
  // Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic> bb;
  // cv::cv2eigen(image, bb); 
  // for(auto i(0); i< bb.cols(); i++){
  //   SingularSpectrumAnalysis ssa(8, 8, false);
  //   ssa.init(bb.col(i));
  //   auto f = ssa.execute();
  //   ssa.save_data(i);
  //   // std::cout<< f.transpose() << std::endl;
  //   ssa.save_vec(f, "smoothed_signal_"+ std::to_string(i));
  // }

  // Eigen::MatrixXf SingularSpectrumAnalysis::get_smoothed_image(cv::Mat& image){
  // Eigen::MatrixXf smoothed_image(image.rows, image.cols);
  // Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> eigen_image;
  // cv::cv2eigen(image, eigen_image); 
  // for(auto i(0); i< eigen_image.rows(); i++){
  //     kamaz::hagen::SingularSpectrumAnalysis ssa(16, 3, true);
  //     ssa.init(eigen_image.row(i));
  //     smoothed_image.row(i) = ssa.execute();
  // }
// }


 
//   // Create a VideoCapture object and open the input file
//   // If the input is the web camera, pass 0 instead of the video file name
//   VideoCapture cap("/dataset/images/trdd8.mp4"); 
    
//   // Check if camera opened successfully
//   if(!cap.isOpened()){
//     cout << "Error opening video stream or file" << endl;
//     return -1;
//   }
//   int counter = 0;
//   int couu =0;
//   while(1){
 
//     Mat frame;
//     // Capture frame-by-frame
//     cap >> frame;
  
//     // If the frame is empty, break immediately
//     if (frame.empty())
//       break;
 
//     // Display the resulting frame
//     // if(counter%3 == 0){
//       std::string image_path = "/dataset/images/result/11/" + std::to_string(couu) + "_lidar_projected.jpg";
//       cv::imwrite(image_path, frame);
//       couu++;
//     // }
//     counter++;
//     // Press  ESC on keyboard to exit
//     char c=(char)waitKey(25);
//     if(c==27)
//       break;
//   }
  
//   // When everything done, release the video capture object
//   cap.release();
 
//   // Closes all the frames
//   destroyAllWindows();
     
//   return 0;
// }

typedef Eigen::Spline<float, 3> Spline3d;


int main(){
  // std::vector<Eigen::VectorXf> waypoints;
  // Eigen::Vector3f po1(2,3,4);
  // Eigen::Vector3f po2(2,5,4);
  // Eigen::Vector3f po3(2,8,9);
  // Eigen::Vector3f po4(2,8,23);
  // waypoints.push_back(po1);
  // waypoints.push_back(po2);
  // waypoints.push_back(po3);
  // waypoints.push_back(po4);
      
  // // The degree of the interpolating spline needs to be one less than the number of points
  // // that are fitted to the spline.
  // Eigen::MatrixXf points(3, waypoints.size());
  // int row_index = 0;
  // for(auto const way_point : waypoints){
  //     points.col(row_index) << way_point[0], way_point[1], way_point[2];
  //     row_index++;
  // }
  // Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2);
  // float time_ = 0;
  // for(int i=0; i<20; i++){
  //     time_ += 1.0/(20*1.0);
  //     Eigen::VectorXf values = spline(time_);
  //     std::cout<< values << std::endl;
  // }
  // std::cout << "Nodes: " << 20 << std::endl;
// std::cout << "Total length: " << smoothed_trajectory.size() << std::endl;

  // time_ = 0;
  // int count = 0;
  // std::vector<float> edges; 
  // for(int g=0; g< 20; g++){
  //   time_ += 1.0/(20*1.0);
  //   Eigen::Vector3f values = spline(time_);
  //   std::cout<< values << std::endl;
  //   edges.push_back(values[0]);
  //   edges.push_back(values[1]);
  //   edges.push_back(values[2]);
  //   // edges.push_back(values[3]);
  //   count += 1;
  // }
  // cnpy::npy_save("file_name.npy", &edges[0],{(unsigned int)1, (unsigned int)count, (unsigned int)3},"w");

  // void TrajectorySmoother::get_smoothed_trajectory(std::vector<Eigen::VectorXf> waypoints
  //                                           , int _number_of_steps
  //                                           , std::vector<Eigen::VectorXf>& smoothed_trajectory){
                
  //               if(waypoints.size()<2){
  //                   smoothed_trajectory = waypoints;
  //                   return;
  //               }
  //               Eigen::MatrixXf points(3, waypoints.size());
  //               // points.col(0) << waypoints[0][0], waypoints[0][1], waypoints[0][2];

  //               int row_index = 0;
  //               for(auto const way_point : waypoints){
  //                   points.col(row_index) << way_point[0], way_point[1], way_point[2];
  //                   std::cout<< "inpuse :-->" << way_point << std::endl;
  //                   row_index++;
  //               }
  //               // points.col(row_index) << waypoints.back()[0], waypoints.back()[1], waypoints.back()[2];

  //               Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2);

  //               // _curve->add_way_point(Vector(waypoints.back()[0], waypoints.back()[1], waypoints.back()[2]));
  //               float time_ = 0;
  //               for(int i=0; i<_number_of_steps; i++){
  //                   time_ += 1.0/(_number_of_steps*1.0);
  //                   Eigen::VectorXf values = spline(time_);
  //                   std::cout<< values << std::endl;
  //                   smoothed_trajectory.push_back(values);
  //               }
  //               std::cout << "Nodes: " << _number_of_steps << std::endl;
	//             std::cout << "Total length: " << smoothed_trajectory.size() << std::endl;
  //           }

  
  std::string filename ="/tmp/fff_946685000909756.pcd"; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud1) == -1) // load point cloud file
  {
      PCL_ERROR("Could not read the file");
      return 0;
  }
  std::cout<<"Loaded"<<cloud1->width * cloud1->height
            <<"data points from /tmp/fff_946685000909756.pcd with the following fields: "
            <<std::endl;

  pcl::PointCloud<pcl::PointXYZ> pc = *cloud1;
  Eigen::Matrix4f sensor_to_world_rotation = Eigen::Matrix4f::Identity();
  
  auto in_cloud = boost::make_shared<pcl::PointCloud<PCLPoint>>(pc);
  kamaz::hagen::Cloud::Ptr cloud_ptr_current_ptr;
   
   kamaz::hagen::DepthGroundRemover* depth_ground_remover;
   std::unique_ptr<kamaz::hagen::ProjectionParams> proj_params_ptr;
   proj_params_ptr = kamaz::hagen::ProjectionParams::VLP_16(10);
   depth_ground_remover = new kamaz::hagen::DepthGroundRemover(*proj_params_ptr);

    cloud_ptr_current_ptr.reset(new kamaz::hagen::Cloud());
        
    cloud_ptr_current_ptr->point_cloud_ground_plane.reset(new pcl::PointCloud<PCLPoint>());
    cloud_ptr_current_ptr->point_cloud_non_ground_plane.reset(new pcl::PointCloud<PCLPoint>());
    cloud_ptr_current_ptr->point_cloud_ptr.reset(new pcl::PointCloud<PCLPoint>());
    cloud_ptr_current_ptr->point_cloud_ptr = in_cloud;
    try{
      cloud_ptr_current_ptr->InitProjection(*proj_params_ptr);
    }catch (const std::length_error& le) {
      std::cerr << FBLU("Error:point cloud is empty...") << le.what() << std::endl;;
      return 0;
    }
    BOOST_LOG_TRIVIAL(info) << FCYN("Number of points in the cloud") << cloud_ptr_current_ptr->point_cloud_ptr->points.size();
    depth_ground_remover->options.bin_size = 7;
    depth_ground_remover->options.ground_remove_angle = 10;
    depth_ground_remover->options.step = 5;
    depth_ground_remover->options.depth_threshold = 1.0f;
    depth_ground_remover->options.window_size = 7;
    depth_ground_remover->options.kernel_size = 7;
    // depth_ground_remover->options.depth_expiration_time = 1.0;
    depth_ground_remover->execute<kamaz::hagen::Cloud::Ptr>(cloud_ptr_current_ptr, 0);


  // //   pcl::io::savePCDFileASCII ("/tmp/diksha_cloud_1.pcd", *(cloud_ptr_current_ptr->point_cloud_ptr));
  // //   pcl::io::savePCDFileASCII ("/tmp/diksha_cloud_2.pcd", *(cloud_ptr_current_ptr->point_cloud_ptr));
  // //   pcl::io::savePCDFileASCII ("/tmp/diksha_cloud_3.pcd", *(cloud_ptr_current_ptr->point_cloud_ptr));

  // //   pcl::PointCloud<pcl::PointXYZ>::Ptr diksha_cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
  // //   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/tmp/diksha_cloud_1.pcd", *diksha_cloud_1) == -1) //* load the file
  //   {
  //     PCL_ERROR ("Couldn't read file /tmp/diksha_cloud_1.pcd \n");
  //     return (-1);
  //   }

  //   pcl::PointCloud<pcl::PointXYZ>::Ptr diksha_cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
  //   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/tmp/diksha_cloud_2.pcd", *diksha_cloud_2) == -1) //* load the file
  //   {
  //     PCL_ERROR ("Couldn't read file /tmp/diksha_cloud_2.pcd \n");
  //     return (-1);
  //   }

  //   pcl::PointCloud<pcl::PointXYZ>::Ptr diksha_cloud_3 (new pcl::PointCloud<pcl::PointXYZ>);
  //   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/tmp/diksha_cloud_3.pcd", *diksha_cloud_3) == -1) //* load the file
  //   {
  //     PCL_ERROR ("Couldn't read file /tmp/diksha_cloud_3.pcd \n");
  //     return (-1);
  //   }

    
  //  kamaz::hagen::Cloud::Ptr cloud_ptr_current;
  //  cloud_ptr_current.reset(new kamaz::hagen::Cloud());
  //  cloud_ptr_current->point_cloud_ground_plane.reset(new pcl::PointCloud<PCLPoint>());
  //  cloud_ptr_current->point_cloud_non_ground_plane.reset(new pcl::PointCloud<PCLPoint>());
  //  cloud_ptr_current->point_cloud_ptr.reset(new pcl::PointCloud<PCLPoint>());

  //   *(cloud_ptr_current->point_cloud_ground_plane) += *(diksha_cloud_1);
  //   *(cloud_ptr_current->point_cloud_non_ground_plane) += *(diksha_cloud_1);
  //   *(cloud_ptr_current->point_cloud_ptr) += *(diksha_cloud_1);

  //   *(cloud_ptr_current->point_cloud_ground_plane) += *(diksha_cloud_2);
  //   *(cloud_ptr_current->point_cloud_non_ground_plane) += *(diksha_cloud_2);
  //   *(cloud_ptr_current->point_cloud_ptr) += *(diksha_cloud_2);

  //   *(cloud_ptr_current->point_cloud_ground_plane) += *(diksha_cloud_3);
  //   *(cloud_ptr_current->point_cloud_non_ground_plane) += *(diksha_cloud_3);
  //   *(cloud_ptr_current->point_cloud_ptr) += *(diksha_cloud_3);
    
}

// int main()
// {
//     kamaz::hagen::RRTStar3D rrtstart3d;
//     kamaz::hagen::CommonUtils common_utils;
//     Eigen::VectorXf x_dimentions(3);
//     x_dimentions << 100, 100, 100;
//     auto map_dim = rrtstart3d.get_search_space_dim(x_dimentions);
//     // auto obstacles = rrtstart3d.get_obstacles();
//     auto obstacles = rrtstart3d.get_random_obstacles(70, x_dimentions);
//     // std::cout<< "-----1" << std::endl;
//     Eigen::VectorXf x_init(3);
//     x_init << 0, 0, 0 ;
//     Eigen::VectorXf x_goal(3);
//     x_goal << 89, 99, 99;

//     std::atomic_bool planner_status;
//     planner_status = ATOMIC_VAR_INIT(true);
//     std::vector<Eigen::VectorXf> Q;
//     Eigen::VectorXf dim_in(2);
//     dim_in << 8, 4;
//     Q.push_back(dim_in);
//     // std::cout<< "-----1" << std::endl;
//     int r = 1;
//     int max_samples = 1000;
//     int rewrite_count = 32;
//     float proc = 0.1;
//     float obstacle_width = 0.5;
//     kamaz::hagen::SearchSpace X;
//     X.init_search_space(map_dim, max_samples, obstacle_width, 0.0, 200, 0.1);
//     // X.insert_obstacles(obstacles);
//     X.update_obstacles_map(obstacles);
//     int save_data_index = 0;
//     rrtstart3d.rrt_init(Q, max_samples, r, proc, rewrite_count);

//     std::vector<SearchSpace::Rect> current_desired_trajectory;
   
//     current_desired_trajectory.push_back(SearchSpace::Rect(0
//                                ,0, 0
//                                , obstacle_width
//                                , obstacle_width
//                                , obstacle_width));
//     current_desired_trajectory.push_back(SearchSpace::Rect(25
//                                ,25, 25
//                                , 25+obstacle_width
//                                , 25+obstacle_width
//                                , 25+obstacle_width));
//     current_desired_trajectory.push_back(SearchSpace::Rect(89
//                                ,99, 99
//                                , 89+obstacle_width
//                                , 99+obstacle_width
//                                , 99+obstacle_width));

//     Eigen::VectorXf center = (x_goal - x_init);
//     Eigen::MatrixXf covmat = Eigen::MatrixXf::Zero(3,3);
//     covmat(0,0) = 40;
//     covmat(1,1) = 40;
//     covmat(2,2) = 40;
    
//     center = (x_goal + x_init)/2;
//     // Eigen::Vector3f a(1,0,0);
//     // Eigen::Vector3f b = x_goal-x_init;
//     // Eigen::Matrix3f rotation_matrix;
//     // common_utils.get_roration_matrix(a, b, rotation_matrix);
//     // Eigen::Quaternion<double> q;
//     // q = rotation_matrix.cast <double>();

//     Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity(3,3);
//     int ndims = covmat.rows();       
//     Eigen::MatrixXf random_points = Eigen::MatrixXf::Zero(max_samples, ndims);
//     common_utils.generate_samples_from_ellipsoid(covmat, rotation_matrix, center
//             , random_points);

//     std::cout<< random_points << std::endl;

//     X.use_whole_search_sapce = true;
//     X.generate_search_sapce(covmat, rotation_matrix, center, max_samples);

//     auto path = rrtstart3d.rrt_planner_and_save(X, x_init, x_goal, x_init, 0.5, 0.5, common_utils, 
//     std::ref(planner_status), save_data_index);
//     Curve* bspline_curve = new BSpline();
// 	bspline_curve->set_steps(100);
//     bspline_curve->add_way_point(Vector(path[0][0], path[0][1], path[0][2]));
//     for(auto const way_point : path){
//       std::cout<<"main: "<< way_point.transpose() << std::endl;
//       bspline_curve->add_way_point(Vector(way_point[0], way_point[1], way_point[2]));
//     }
//     bspline_curve->add_way_point(Vector(path.back()[0], path.back()[1], path.back()[2]));
//     std::cout << "nodes: " << bspline_curve->node_count() << std::endl;
// 	std::cout << "total length: " << bspline_curve->total_length() << std::endl;
//     std::vector<Eigen::VectorXf> new_path_bspline;
//     if(path.size()>0){
//       new_path_bspline.push_back(path[0]);
//     }
//     for (int i = 0; i < bspline_curve->node_count(); ++i) {
// 	    Eigen::VectorXf pose(3);
//         auto node = bspline_curve->node(i);
//         pose<< node.x, node.y, node.z; 
//         new_path_bspline.push_back(pose);
// 	}
//     std::string path_ingg = "/dataset/rrt_old/" + std::to_string(save_data_index) + "_rrt_path_modified.npy";
//     rrtstart3d.save_path(new_path_bspline, path_ingg);

    
//     save_data_index++;
//     rrtstart3d.rrt_init(Q, max_samples, r, proc, rewrite_count);
//     X.use_whole_search_sapce = false;
//     X.insert_trajectory(current_desired_trajectory);
//     path = rrtstart3d.rrt_planner_and_save(X, x_init, x_goal, x_goal, 2.0, 3.0, common_utils, 
//     std::ref(planner_status), save_data_index);
//     bspline_curve = new BSpline();
// 	  bspline_curve->set_steps(100);
//     bspline_curve->add_way_point(Vector(path[0][0], path[0][1], path[0][2]));
//     for(auto const way_point : path){
//       std::cout<<"main: "<< way_point.transpose() << std::endl;
//       bspline_curve->add_way_point(Vector(way_point[0], way_point[1], way_point[2]));
//     }
//     bspline_curve->add_way_point(Vector(path.back()[0], path.back()[1], path.back()[2]));
//     std::cout << "nodes: " << bspline_curve->node_count() << std::endl;
// 	std::cout << "total length: " << bspline_curve->total_length() << std::endl;
//     new_path_bspline.clear();
//     if(path.size()>0){
//       new_path_bspline.push_back(path[0]);
//     }
//     for (int i = 0; i < bspline_curve->node_count(); ++i) {
// 	    Eigen::VectorXf pose(3);
//         auto node = bspline_curve->node(i);
//         pose<< node.x, node.y, node.z; 
//         new_path_bspline.push_back(pose);
// 	}
//     path_ingg = "/dataset/rrt_old/" + std::to_string(save_data_index) + "_rrt_path_modified.npy";
//     rrtstart3d.save_path(new_path_bspline, path_ingg);

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