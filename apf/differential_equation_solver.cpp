#include "differential_equation_solver.h"

namespace kamaz
{
    namespace hagen {
    std::unique_lock<std::mutex> mutex_worker;
    APFCalculator apfCalculator;
    static int counter = 0;
    bool differentiator_processed = false;

    DifferentialEquationSolver::DifferentialEquationSolver(){

    };
    
    struct streaming_observer
    {
        std::ostream &m_out;
        std::vector<tuple<double, double, double>> &projected_trajectory;
        streaming_observer( std::ostream &out , std::vector<tuple<double, double, double>>& projected_trajectory ) : m_out( out ), projected_trajectory(projected_trajectory) {}

        void operator()( const state_type &x , double t ) const
        {
            m_out << t;
            for( size_t i=0 ; i < x.size() ; ++i )
                m_out << "\t\t" << x[i];
            m_out << "\n";

            // if(!differentiator_processed and apfCalculator.isTerminate(x) and counter%40){
            //     differentiator_processed = true;
            //     mutex_worker.unlock();
            //     differentiator_cv.notify_one();
            // }
            
            // if(!differentiator_processed){
            //     std::tuple<double, double, double> next_position(x[0], x[1], x[2]);
            //     // apfCalculator.addToTrajectory(next_position);
            //     projected_trajectory.push_back(next_position);
            //     std::cout<< "------------->>" << projected_trajectory.size() << std::endl;
            // }

            if(apfCalculator.isTerminate(x) and counter%40){
                differentiator_processed = true;
            }
            
            if(!differentiator_processed){
                std::tuple<double, double, double> next_position(x[0], x[1], x[2]);
                // apfCalculator.addToTrajectory(next_position);
                std::cout<< x[0] <<"," << x[1] << "," << x[2] << std::endl;
                projected_trajectory.push_back(next_position);
            }
            counter++;
        }
    };

    struct dq_solver
    {
        dq_solver( ) {};
        void operator()(const state_type &stateValue, state_type &dxdt, const double t) {
            state_type result  = apfCalculator.navigationFunction(stateValue);
            dxdt[0]= result[0];
            dxdt[1]= result[1];
            dxdt[2]= result[2];
        }
    };

    void DifferentialEquationSolver::init(APFCalculator& apfCalculatorInstance){
        //mutex_worker = std::unique_lock<std::mutex>(differentiator_lock);
        apfCalculator = apfCalculatorInstance;
    }

    bool DifferentialEquationSolver::run(std::vector<tuple<double, double, double>>& projected_trajectory)
	{
        //differentiator_cv.wait(mutex_worker, []{return differentiator_ready;});
        state_type stateValue;
        stateValue[0] = apfCalculator.qStart.row(0).at<double>(0);
        stateValue[1] = apfCalculator.qStart.row(0).at<double>(1);
        stateValue[2] = apfCalculator.qStart.row(0).at<double>(2);
        const double dt = apfCalculator.stepSize;
        const double t_start = 0.0;
        double t = t_start;
        runge_kutta4<state_type> rk4;
        runge_kutta_dopri5<state_type> rkd5;
        integrate_adaptive( stepper_type() , dq_solver() , stateValue , 0.0 , 900000.0 , dt,
                        streaming_observer( cout , projected_trajectory ));
        // if(!differentiator_processed){
        //     differentiator_processed = true;
        //     // mutex_worker.unlock();
        //     // differentiator_cv.notify_one(); 
        // }
        return differentiator_processed;  
    }
    }
}
