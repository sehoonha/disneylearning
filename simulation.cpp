#include "simulation.h"

#include "cppcommon.h"

namespace disneysimple {
namespace sim {

Simulation::Simulation() {
    init();
    // step();

    // Eigen::VectorXd state = Eigen::VectorXd::Zero(12);
    // state(0) = 0.01;
    // state(1) = 0.02;
    // Eigen::VectorXd control = Eigen::VectorXd::Zero(6);
    // deriv(state, control);
}

Simulation::~Simulation() {
}

void Simulation::init() {
    int n = 6;
    int m = 2 * n;
    Eigen::VectorXd xEq(m);
    xEq << 0, 0, 0, 0, PI_2, -PI_2
         , 0, 0, 0, 0, 0, 0;

    Eigen::VectorXd xOffset(m);
    double angIni = 2;
    xOffset << 0.0, (angIni * PI / 180), 0, 0, 0, 0
        , 0, 0, 0, 0, 0, 0;
        
    mState = xEq + xOffset;

    mStateHistory.clear();
    mStateHistory.push_back(mState);
}

void Simulation::control() {
    // Zero control, for now
    int n = nDimConfig();
    mTorque = Eigen::VectorXd::Zero(n);


    // Parameters
    double radius = 0.05;
    double rw     = radius;
    double lrl1 = 1;
    double lrl2 = 0.1;
    double lll1 = 1;
    double lll2 = 0.1;
    double maxTorq = 200; 


    // Feedback Matrix
    Eigen::MatrixXd C(5, 2 * n);
    C << - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,  
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,    
        1, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0;  


    Eigen::MatrixXd F(4, 5);
    F << -2716691.61124073, -1189377.26019503, 953603.332318603, 10071.8805576070, 768507.689768547,
        -2716691.61123261, -1189377.26019358, 953603.332315551, 10071.8805576529, 768507.689765388,
        2716691.61123813, 1189377.26019453, -953603.332317613, -10071.8805576191, -768507.689767548,
        2716691.61124318, 1189377.26019541, -953603.332319511, -10071.8805575885, -768507.689769501;

    Eigen::MatrixXd K = F * C;

    // State 
    Eigen::VectorXd x = mState;
    
    // Equilibrium state
    Eigen::VectorXd qEq(n);
    qEq << 0.0, 0.0, 0.0, 0.0, PI/2.0, -PI/2.0;
    Eigen::VectorXd dqEq(n);
    dqEq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd xEq(n * 2);
    xEq.head(n) = qEq;
    xEq.tail(n) = dqEq;
        
    // Calculate u
    Eigen::VectorXd u = -K * (x - xEq);
    for(int i = 0; i < u.size(); i++) {
        if (fabs(u(i)) > maxTorq) {
            if (u(i) > 0) {
                u(i) = maxTorq;
            } else {
                u(i) = -maxTorq;
            }
        }
    }
    mTorque = u;

}

void Simulation::step() {
    control();
    integrate();
}

/*
// This function's implementation is moved to simulation_deriv.cpp
Eigen::VectorXd
Simulation::deriv(const Eigen::VectorXd& x, const Eigen::VectorXd& control)
*/

/*
  the fourth order Runge-Kutta method
*/
void Simulation::integrate() {
    double h = 0.001;
    double h_2 = 0.5 * h;

    Eigen::VectorXd x = mState;
    Eigen::VectorXd u = mTorque;

    // LOG(INFO) << "x = " << x.transpose();
    // LOG(INFO) << "u = " << u.transpose();

    Eigen::VectorXd k1 = deriv(x, u);
    Eigen::VectorXd k2 = deriv(x + h_2 * k1, u);
    Eigen::VectorXd k3 = deriv(x + h_2 * k2, u);
    Eigen::VectorXd k4 = deriv(x + h * k3, u);

    Eigen::VectorXd dx = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
    // LOG(INFO) << "dx = " << dx.transpose();

    mState = x + h * dx;
    mStateHistory.push_back(mState);
}


} // namespace sim
} // namespace disneysimple



