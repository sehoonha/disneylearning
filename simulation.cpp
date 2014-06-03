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
    double angIni = 0.03;
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



