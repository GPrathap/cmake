#include "dynamics.h"

namespace kamaz {
namespace hagen{


    Dynamics::Dynamics(Eigen::MatrixXd xStartState, Eigen::MatrixXd xGoalState){
        gravity = 9.80665; // gravity,   m/s^2
        dt = 0.5;
        // xDim = 13;
        uDim  = 4;
        jStep = 0.0009765625;
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
        Q = 500*Eigen::MatrixXd::Identity(xDim, xDim);
        // Q(X_DIM-1,X_DIM-1) = 0;
        rotCost = 0.3; 
        obstacleFactor = 1;
        scaleFactor = 10;
        R = 20*Eigen::MatrixXd::Identity(uDim, uDim);
        // Environment parameters
        robotRadius = length + 0.1;
        
        Q = Eigen::MatrixXd::Zero(xDim, xDim);
        xGoal = xGoalState;
        xStart = xStartState;
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

    Eigen::MatrixXd Dynamics::gBar(const Eigen::MatrixXd x, const Eigen::MatrixXd u, double dt)
    {
        Eigen::MatrixXd k1 = f(x, u);
        Eigen::MatrixXd k2 = f(x - 0.5*dt*k1, u);
        Eigen::MatrixXd k3 = f(x - 0.5*dt*k2, u);
        Eigen::MatrixXd k4 = f(x - dt*k3, u);
        return x - (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
    }

    Eigen::MatrixXd Dynamics::jacobian2(Eigen::MatrixXd& a, Eigen::MatrixXd& b) {
        Eigen::MatrixXd B(xDim, uDim);
        Eigen::MatrixXd br = b;
        Eigen::MatrixXd bl = b;
        for (size_t i = 0; i < uDim; ++i) {
            br(i) += jStep;
            bl(i) -= jStep;
            B.block<4, 1>(0,i) = (gBar(a, br, dt) - gBar(a, bl, dt)) / (2*jStep);
            br(i) = bl(i) = b(i);
        }
	    return B;
    }


    Eigen::MatrixXd Dynamics::jacobian1(Eigen::MatrixXd& a, Eigen::MatrixXd& b) {
        Eigen::MatrixXd A(xDim, xDim);
        Eigen::MatrixXd ar = a;
        Eigen::MatrixXd al = a;
        for (int i = 0; i < xDim; ++i) {
            ar(i) += jStep;
            al(i) -= jStep;
            Eigen::MatrixXd val = (gBar(ar, b, dt) - gBar(al, b, dt)) / (2*jStep);
            A.block<13, 1>(0,i) = val;
            ar(i) = al(i) = a(i);
        }
        return A;
    }

    double Dynamics::extendedLQR(const int ell, Eigen::MatrixXd& startState
        , Eigen::MatrixXd& uNominal, std::vector<Eigen::MatrixXd>& L, std::vector<Eigen::MatrixXd>& l) {

        size_t maxIter = 10;
        L.resize(ell, Eigen::MatrixXd::Zero(xDim, uDim));
        l.resize(ell, Eigen::MatrixXd::Zero(ell, uDim));
        std::vector<Eigen::MatrixXd> lx_state;
        //TODO need to fix symmetric
        std::vector<Eigen::MatrixXd> S(ell + 1, Eigen::MatrixXd::Zero(xDim, 1));
        std::vector<Eigen::MatrixXd> s(ell + 1, Eigen::MatrixXd::Zero(xDim, 1));
        std::vector<Eigen::MatrixXd> SBar(ell + 1);
        std::vector<Eigen::MatrixXd> sBar(ell + 1);

	    double oldCost = -log(0.0);
	    Eigen::MatrixXd xHat = startState;

        SBar[0] = Eigen::MatrixXd::Zero(xDim, 1);
        sBar[0] = Eigen::MatrixXd::Zero(xDim, 1);
    
		for (size_t t = 0; t < ell; ++t) {
		// 	// std::cout<< "==========L[t]====\n"<< L[t] << std::endl;
			
		// 	// std::cout<< "========l[t]======\n"<< l[t]  << std::endl;
			Eigen::MatrixXd uHat = L[t]*xHat + l[t];
		// 	std::cout<< "======l[t] "<< l[t] << std::endl;
			Eigen::MatrixXd xHatPrime = g(xHat, uHat);
			
			Eigen::MatrixXd ABar = jacobian1(xHatPrime, uHat);
			Eigen::MatrixXd BBar = jacobian2(xHatPrime, uHat);
			Eigen::MatrixXd cBar = xHat - ABar*xHatPrime - BBar*uHat;

			Eigen::MatrixXd P;
			Eigen::MatrixXd Q(xDim, xDim);
			Eigen::MatrixXd R(uDim, uDim);
			Eigen::MatrixXd q(xDim, 1);
			Eigen::MatrixXd r(xDim, 1);

			quadratizeCost(xHat, uHat, t, P, Q, R, q, r, 0);

			Eigen::MatrixXd SBarQ = SBar[t] + Q;
			Eigen::MatrixXd sBarqSBarQcBar = sBar[t] + q + SBarQ*cBar;

			Eigen::MatrixXd CBar = BBar.transpose()*SBarQ*ABar + P*ABar;
			Eigen::MatrixXd DBar = ABar.transpose()*(SBarQ*ABar);
			// std::cout<< "--------->" << std::endl;
            Eigen::MatrixXd Pbbr = P*BBar;
			Eigen::MatrixXd EBar = BBar.transpose()*(SBarQ*BBar) + R + (Pbbr + Pbbr.transpose());
			// std::cout<< "--------->" << std::endl;
			Eigen::MatrixXd dBar = ABar.transpose()*sBarqSBarQcBar;
			Eigen::MatrixXd eBar = BBar.transpose()*sBarqSBarQcBar + r + P*cBar;
            Eigen::MatrixXd EBarInv = EBar.inverse();
			L[t] = -1*EBarInv*CBar;
			l[t] = -1*EBarInv*eBar;

            Eigen::MatrixXd CbarTrans = CBar.transpose();

			SBar[t+1] = DBar + CbarTrans*L[t];
			sBar[t+1] = dBar + CbarTrans*l[t];

			xHat = -((S[t+1] + SBar[t+1]).inverse()*(s[t+1] + sBar[t+1]));

			// xHat = -((S[t+1] + SBar[t+1])%(s[t+1] + sBar[t+1]));
		// 	// std::cout<< "======129"<< t << std::endl;
		// 	lx_state.push_back(xHat);
		// 	// std::cout<< "==========xHat====\n"<< xHat << std::endl;
		// 	// std::cout<< "========l[t] ======\n"<< l[t]  << std::endl;
		// 	std::cout<< "======== xHat ======\n"<< xHat  << std::endl;
		}

		// std::vector<double> sates_sequeance; 
		// int count = lx_state.size();
		// for(auto item : lx_state){
		// 	for(int i=0; i<xDim; i++){
		// 		sates_sequeance.push_back(item[i]);
		// 	}		
		// }

		// std::vector<double> control_sequeance; 
		// count = l.size();
		// for(auto item : l){
		// 	for(int i=0; i<uDim; i++){
		// 		control_sequeance.push_back(item[i]);
		// 	}		
		// }

        // cnpy::npy_save("/home/geesara/Desktop/ler/0_state_vector.npy", &sates_sequeance[0],{(unsigned int)1, (unsigned int)count, (unsigned int)xDim},"w");
        // cnpy::npy_save("/home/geesara/Desktop/ler/0_control_vector.npy", &control_sequeance[0],{(unsigned int)1, (unsigned int)count, (unsigned int)uDim},"w");


		// backward pass
		// quadratizeFinalCost(xHat, S[ell], s[ell], iter);
		// xHat = -((S[ell] + SBar[ell])%(s[ell] + sBar[ell]));
        
		// for (size_t t = ell - 1; t != -1; --t) {
		// 	const Matrix<uDim> uHat = L[t]*xHat + l[t];
		// 	const Matrix<xDim> xHatPrime = gBar(xHat, uHat);
		// 	const Matrix<xDim, xDim> A = jacobian1(xHatPrime, uHat, g);
		// 	const Matrix<xDim, uDim> B = jacobian2(xHatPrime, uHat, g);
		// 	const Matrix<xDim> c = xHat - A*xHatPrime - B*uHat;
		// 	Matrix<uDim, xDim> P;
		// 	SymmetricMatrix<xDim> Q;
		// 	SymmetricMatrix<uDim> R;
		// 	Matrix<xDim> q;
		// 	Matrix<uDim> r;
		// 	quadratizeCost(xHatPrime, uHat, t, P, Q, R, q, r, iter);
		// 	const Matrix<uDim,xDim> C = ~B*S[t+1]*A + P;
		// 	const SymmetricMatrix<xDim> D = SymProd(~A,S[t+1]*A) + Q;
		// 	const SymmetricMatrix<uDim> E = SymProd(~B,S[t+1]*B) + R;
		// 	const Matrix<xDim> d = ~A*(s[t+1] + S[t+1]*c) + q;
		// 	const Matrix<uDim> e = ~B*(s[t+1] + S[t+1]*c) + r;
        //     std::cout<< "======123"<< E << std::endl;
		// 	std::cout<< "======123"<< C << std::endl;
		// 	L[t] = -(E%C);
		// 	l[t] = -(E%e);
		// 	S[t] = D + SymProd(~C, L[t]);
		// 	s[t] = d + ~C*l[t];
		// 	xHat = -((S[t] + SBar[t])%(s[t] + sBar[t]));
		// }

		// // std::cout<< "====" << xHat <<  "====" << std::endl;
        
		// // compute cost
		// double newCost = 0;
		// Matrix<xDim> x = xHat;
		// for (size_t t = 0; t < ell; ++t) {
		// 	Matrix<uDim> u = L[t]*x + l[t];
		// 	newCost += ct(x, u, t);
		// 	x = g(x, u);
		// }
		//         std::cout<< "======123" << std::endl;

		// newCost += cell(x);

		// // if (vis) {
		// // 	std::cerr << "Iter: " << iter << " Rel. progress: " << (oldCost - newCost) / newCost << " Cost: " << newCost << " Time step: " << exp(xHat[xDim-1]) << std::endl;
		// // }
		// if (abs((oldCost - newCost) / newCost) < 1e-4) {
		// 	++iter;
		// 	return exp(xHat[xDim-1]);
		// }
		// oldCost = newCost;
	// }

	    return 0.0;
    }

    void Dynamics::quadratizeCost(Eigen::MatrixXd& x, Eigen::MatrixXd& u, const size_t& t
        , Eigen::MatrixXd& Pt, Eigen::MatrixXd& Qt, Eigen::MatrixXd& Rt, Eigen::MatrixXd& qt
        , Eigen::MatrixXd& rt, const size_t& iter) {

        Qt = Q;
        qt = -(Q*xStart);
        Qt(xDim-1, xDim-1) = 100;
        qt(xDim-1) = -Qt(xDim-1,xDim-1)*x(xDim-1);

        Rt = R; 
        rt = -(R*uNominal);
        // Pt.reset();
    }
}
}