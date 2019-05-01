#ifndef PERACOPTER_DFFIAPFCALCULATOR_H
#define PERACOPTER_DFFIAPFCALCULATOR_H

#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <vector>
#include <functional>
#include <boost/numeric/odeint.hpp>
#include <utility>
#include <algorithm>
#include <array>

#include <thread>
#include <assert.h>
#include <chrono>
#include <future>

#include "newTypeDef.h"
#include "apf_calculator.h"

using namespace std;
using namespace cv;
namespace odeint = boost::numeric::odeint;

namespace kamaz  {
    namespace hagen {
    class DifferentialEquationSolver{
    public:
        DifferentialEquationSolver();
        ~DifferentialEquationSolver() = default;
        typedef controlled_runge_kutta< runge_kutta_cash_karp54< state_type > > stepper_type;
        //void operator()( state_type &x , state_type &dxdt , double t );
        //void operator()(const state_type &x, state_type &dxdt, const double t);
        template<class System, class Condition> state_type find_condition(state_type &x0,
              System sys, Condition cond, const double t_start, const double t_end, const double dt,
              const double precision = 1E-6);
        void init(APFCalculator& apfCalculatorInstance);
        bool run(std::vector<tuple<double, double, double>>& projected_trajectory);
    };
    }
}
#endif
