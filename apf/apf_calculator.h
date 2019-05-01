#ifndef PERACOPTER_APFCALCULATOR_H
#define PERACOPTER_APFCALCULATOR_H

#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <vector>
#include <functional>
#include <boost/numeric/odeint.hpp>
#include "newTypeDef.h"
#include "../rrt_star/search_space.h"

using namespace std;
using namespace cv;
using namespace boost::numeric::odeint;

namespace kamaz
{
    namespace hagen {

    class APFCalculator{
    public:
        APFCalculator();
        ~APFCalculator();
        bool initArena(std::vector<Mat> obstacles);
        bool initPositions(Mat start, Mat end);
        state_type navigationFunction(state_type);
        state_type calculatingGradientAttractiveRepulsiveForce(state_type);
        Mat rotationmat3D(double, Mat);
        bool isTerminate(state_type);
        //void oneIteration(double time, int neq, std::function<Mat(Mat)> diffFunction);
        //Mat ode45(std::vector<double> , Mat, std::function<Mat(Mat)> diffFunction);
        std::vector<double> linspace(double start_in, double end_in, double num_in);
        //void addToTrajectory(tuple<double, double, double> position);
        std::vector<cv::Mat> get_obstacles(std::vector<SearchSpace::Rect> obs);

        Mat arenaMap;
        Mat qStart;
        Mat qGoal;
        Mat currentPosition;
        double k;
        double obsTh;
        double arenaR;
        double epsilonGoal;
        double goalTh;
        double rotationAngle = M_PI/8;
        Mat rotationVector;
        Mat rotationalMatrix;
        double stepSize;
        bool goalIsFound;
        //vector<tuple<double, double, double>> projected_trajectory;
    };
    }
}

#endif


