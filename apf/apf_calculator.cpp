#include "apf_calculator.h"

namespace kamaz
{
    namespace hagen {

    APFCalculator::APFCalculator()
    {
        arenaR = 150.0;
        stepSize = 0.5;
        k = 2.0;
        obsTh = 10;
        goalTh = 20;
        epsilonGoal = 0.8;
        goalIsFound = false;

//      rotationAngle = M_PI / 4;
//      rotationVector = (Mat_<double>(1, 3) << 1.0, 1.0, 1.0);
//      rotationalMatrix.create(1, 3, DataType<double>::type);
    }

    APFCalculator::~APFCalculator()
    {

    }

    bool APFCalculator::initArena(std::vector<Mat> obstacles){
        arenaMap = Mat::zeros((int)obstacles.size(), 4, DataType<double>::type);
        for(size_t i=0; i< obstacles.size(); i++){
            obstacles.at(i).copyTo(arenaMap.row((int)i));
        }
        return true;
    }

    bool APFCalculator::initPositions(Mat start, Mat end) {
        currentPosition = Mat::zeros(1, 3, DataType<double>::type);
        start.copyTo(qStart);
        qStart.copyTo(currentPosition);
        end.copyTo(qGoal);
        return true;
    }

    std::vector<cv::Mat> APFCalculator::get_obstacles(std::vector<SearchSpace::Rect> obs){
        std::vector<cv::Mat> obstacles_previous_try;
        for(auto const& obstacle: obs){
                obstacles_previous_try.push_back(
                    (cv::Mat)(cv::Mat_<double>(1, 4) << obstacle.min[0], obstacle.min[1], obstacle.min[2], 4));
        }
        return obstacles_previous_try;
    }


    // void APFCalculator::addToTrajectory(std::tuple<double, double, double> position){
    //     projected_trajectory.push_back(position);
    // }

    Mat APFCalculator::rotationmat3D(double rotationAngle, Mat rotationVector) {
        double l = cv::norm(rotationVector);
        rotationVector = rotationVector / l;
        double u = rotationVector.at<double>(0, 0);
        double v = rotationVector.at<double>(0, 1);
        double w = rotationVector.at<double>(0, 2);
        double u2 = pow(u,2);
        double v2 = pow(v, 2);
        double w2 = pow(w ,2);
        double c = cos(rotationAngle);
        double s = sin(rotationAngle);
        Mat rotationMatrix(3, 3, DataType<double>::type);
        rotationMatrix.at<double>(0, 0) = u2 + (v2 + w2) * c;
        rotationMatrix.at<double>(0, 1) = u * v * (1 - c) - w * s;
        rotationMatrix.at<double>(0, 2) = u * w * (1 - c) + v * s;
        rotationMatrix.at<double>(1, 0) = u * v * (1 - c) + w * s;
        rotationMatrix.at<double>(1, 1) = v2 + (u2 + w2) * c;
        rotationMatrix.at<double>(1, 2) = v * w * (1 - c) - u * s;
        rotationMatrix.at<double>(2, 0) = u * w * (1 - c) - v * s;
        rotationMatrix.at<double>(2, 1) = v * w * (1 - c) + u * s;
        rotationMatrix.at<double>(2, 2) = w2 + (u2 + v2) * c;
        return rotationMatrix;
    }

    state_type APFCalculator::navigationFunction(state_type stateValue) {
        Mat robotPosition = (Mat_<double>(1,3) << stateValue[0], stateValue[1], stateValue[2]);
        Mat q = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        Mat qg = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        robotPosition.copyTo(q);
        qGoal.row(0).copyTo(qg);
        double dqqg = cv::norm(q-qg);
        Mat Ddqqg =  (q - qg) / dqqg;
        double Bl = -pow(cv::norm(q), 2.0) + pow(arenaR,2.0);
        double Bq = Bl;
        Mat DB0 = -2.0 * q;
        std::vector<double> Bi;
        std::vector<double> DBi_1;
        std::vector<double> DBi_2;
        std::vector<double> DBi_3;
        for(size_t i=0; i<arenaMap.rows; i++){
            Mat qi = arenaMap.row((int)i).colRange(0,3);
            Bi.push_back(pow(cv::norm(q - qi), 2) - pow(arenaMap.row((int)i).at<double>(3),2));
            Bq = Bq * Bi.at(i);
            DBi_1.push_back(2 * (q.at<double>(0) - qi.at<double>(0)));
            DBi_2.push_back(2 * (q.at<double>(1) - qi.at<double>(1)));
            DBi_3.push_back(2 * (q.at<double>(2) - qi.at<double>(2)));
        }
        Bi.push_back(Bl);
        DBi_1.push_back((double)DB0.at<double>(0));
        DBi_2.push_back((double)DB0.at<double>(1));
        DBi_3.push_back((double)DB0.at<double>(2));
        Mat DBq = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        for(size_t i =0; i<= arenaMap.rows; i++){
            Mat DBi_d = (Mat_<double>(1,3) << DBi_1.at(i), DBi_2.at(i), DBi_3.at(i));
            DBq.row(0).at<double>(0) = DBq.row(0).at<double>(0) + (DBi_d.at<double>(0) * Bq / Bi.at(i));
            DBq.row(0).at<double>(1) = DBq.row(0).at<double>(1) + (DBi_d.at<double>(1) * Bq / Bi.at(i));
            DBq.row(0).at<double>(2) = DBq.row(0).at<double>(2) + (DBi_d.at<double>(2) * Bq / Bi.at(i));
        }
        double p = pow(dqqg,(2.0*k)) + Bq;
        Mat Dp_1k = pow(p,(1.0/k-1.0))/ k*(2.0*k*pow((dqqg),(2.0*k-1.0))*Ddqqg+DBq);
        Mat DelNF = ((2.0*dqqg*Ddqqg)*pow((p),(1.0/k)) - pow((dqqg),(2.0))*Dp_1k)/pow((p),(2.0/k));
        state_type gradient;
        gradient[0]=-DelNF.at<double>(0);
        gradient[1]=-DelNF.at<double>(1);
        gradient[2]=-DelNF.at<double>(2);
        return gradient;
    }

    state_type APFCalculator::calculatingGradientAttractiveRepulsiveForce(state_type stateValue) {

        Mat robotPosition = (Mat_<double>(1,3) << stateValue[0], stateValue[1], stateValue[2]);
        double Kappa = 1.0;
        double Nu = 1.0;
        Mat GUrep = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        Mat GUrep_bnd = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        double nrp = cv::norm(robotPosition);
        double D = arenaR - nrp;
        if (D<=obsTh){
            double Cons = Nu * (1/obsTh - 1/D) * (1/std::pow(D,2));
            GUrep_bnd.at<double>(0) = Cons * (-robotPosition.at<double>(0)/nrp);
            GUrep_bnd.at<double>(1) = Cons * (-robotPosition.at<double>(1)/nrp);
            GUrep_bnd.at<double>(2) = Cons * (-robotPosition.at<double>(2)/nrp);
        }

        Mat GUrep_obs = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        for(int i=0; i<arenaMap.rows; i++){
            D = sqrt(pow((arenaMap.row(i).at<double>(0) - robotPosition.at<double>(0)), 2) +
                             pow((arenaMap.row(i).at<double>(1) - robotPosition.at<double>(1)), 2) +
                             pow((arenaMap.row(i).at<double>(2) - robotPosition.at<double>(2)), 2)) -
                    arenaMap.row(i).at<double>(3);
            if(D<=obsTh){
                double DtoCenter = std::sqrt(std::pow(((double)arenaMap.row(i).at<double>(0) -
                        robotPosition.at<double>(0)), 2) + std::pow(((double)arenaMap.row(i).at<double>(1) -
                        robotPosition.at<double>(1)), 2)+ std::pow(((double)arenaMap.row(i).at<double>(2) -
                        robotPosition.at<double>(2)), 2));
                double Cons = Nu * (1.0/obsTh - 1.0/D) * (1.0/std::pow(D,2));
                GUrep_bnd.at<double>(0) = GUrep_bnd.at<double>(0) + Cons * ((robotPosition.at<double>(0) -
                        arenaMap.row(i).at<double>(0))/DtoCenter); //  x direction
                GUrep_bnd.at<double>(1) = GUrep_bnd.at<double>(1) + Cons * ((robotPosition.at<double>(1) -
                        arenaMap.row(i).at<double>(1))/DtoCenter); // y direction
                GUrep_bnd.at<double>(2) = GUrep_bnd.at<double>(2) + Cons * ((robotPosition.at<double>(2) -
                        arenaMap.row(i).at<double>(2))/DtoCenter); // z direction
            }

        }

        GUrep = GUrep_bnd + GUrep_obs;
        // attractive Potential gradient
        Mat GUatt = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        Mat Dummy = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        Dummy.at<double>(0) = robotPosition.at<double>(0) - qGoal.at<double>(0);
        Dummy.at<double>(1) = robotPosition.at<double>(1) - qGoal.at<double>(1);
        Dummy.at<double>(2) = robotPosition.at<double>(2) - qGoal.at<double>(2);
        double nrmg = cv::norm(Dummy);
        if (nrmg <= goalTh){
            GUatt = Kappa * (Dummy);
        }
        else{
            GUatt = (goalTh * Kappa / nrmg) * (Dummy);
        }
        state_type gradient;
        gradient[0]= (-GUrep.at<double>(0)) + (-GUatt.at<double>(0));
        gradient[1]= (-GUrep.at<double>(1)) + (-GUatt.at<double>(1));
        gradient[2]= (-GUrep.at<double>(2)) + (-GUatt.at<double>(2));
        return gradient;
    }

    bool APFCalculator::isTerminate(state_type robotPosition) {

        Mat Dummy = cv::Mat(1, 3, DataType<double>::type,{0, 0, 0});
        Dummy.at<double>(0) = robotPosition[0] - qGoal.at<double>(0);
        Dummy.at<double>(1) = robotPosition[1] - qGoal.at<double>(1);
        Dummy.at<double>(2) = robotPosition[2] - qGoal.at<double>(2);
        double m = cv::norm(Dummy);
        if(m<=epsilonGoal){
            //printf("Distance to goal: %f\n", m);
            return true;
        }
        return false;
    }

//    void APFCalculator::oneIteration(double time, int neq, std::function<Mat(Mat)> diffFunction) {
//        //Mat f0 = calculatingGradientAttractiveRepulsiveForce(currentPosition);
//        Mat f0 = navigationFunction(currentPosition);
//        //cout<< f0 << endl;
//        Mat A = (Mat_<double>(1,6) << 1.0/5, 3.0/10, 4.0/5, 8.0/9, 1.0, 1.0);
//        Mat B = (Mat_<double>(7,6) <<
//        1.0/5,         3.0/40,    44.0/45,  19372.0/6561,      9017.0/3168,       35.0/384,
//        0.0,           9.0/40,    -56.0/15,  -25360.0/2187,     -355.0/33,         0.0,
//                0.0,           0.0,       32.0/9,    64448.0/6561,      46732.0/5247,      500.0/1113,
//                0.0,           0.0,       0.0,       -212.0/729,        49.0/176,          125.0/192,
//                0.0,           0.0,       0.0,       0.0,               -5103.0/18656,     -2187.0/6784,
//                0.0,           0.0,       0.0,       0.0,               0.0,               11.0/84,
//                0.0,           0.0,       0.0,       0.0,               0.0,               0.0);
//
//        Mat f = Mat::zeros(neq, 7, DataType<double>::type);
//        f0.copyTo(f.col(0));
//        Mat hA = stepSize* A;
//        Mat hB = stepSize* B;
//        for (int i=0; i<5; i++){
//            Mat y_f_hB5 = currentPosition+(f*hB.col(i));
//            Mat gradient = navigationFunction(y_f_hB5);
////            Mat gradient = calculatingGradientAttractiveRepulsiveForce(y_f_hB5);
//            gradient.copyTo(f.col(i+1));
//        }
//        currentPosition = currentPosition + f*hB.col(5);
//    }

//    Mat APFCalculator::ode45(std::vector<double> T, Mat initPosition, std::function<Mat(Mat)> diffFunction) {
//        currentPosition = initPosition;
//        int nT = (int)T.size();
//        int neq = currentPosition.rows;
//        Mat Y = Mat::ones(neq,nT, DataType<double>::type);
//        double t00=0;
//        for(int i=0; i<nT; i++){
//            T.at((unsigned)i)=t00;
//            oneIteration(t00, neq, diffFunction);
//            currentPosition.copyTo(Y.col(i));
//            t00=t00+stepSize;
////            if(isTerminate(currentPosition)){
////                goalIsFound = true;
////                printf("Destination has reached with %d iterations.", i);
////                return Y.colRange(0, i);
////            }
//        }
//        return Y;
//    }


    std::vector<double> APFCalculator::linspace(double start_in, double end_in, double step_size)
    {
        std::vector<double> linspaced;
        double start = static_cast<double>(start_in);
        double end = static_cast<double>(end_in);
        double num = static_cast<double>(step_size);
        if (num == 0) { return linspaced; }
        if (num == 1)
        {
            linspaced.push_back(start);
            return linspaced;
        }
        double delta = (end - start) / (num - 1);
        for(int i=0; i < num-1; ++i)
            {
            linspaced.push_back(start + delta * i);
            }
        linspaced.push_back(end);
        return linspaced;
    }
    }
}
