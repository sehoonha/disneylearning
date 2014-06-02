#include "simulation.h"

#include "cppcommon.h"

namespace disneysimple {
namespace sim {

Simulation::Simulation() {
    init();
    Eigen::VectorXd state = Eigen::VectorXd::Zero(12);
    state(0) = 0.01;
    state(1) = 0.02;
    Eigen::VectorXd control = Eigen::VectorXd::Zero(6);
    deriv(state, control);
}

Simulation::~Simulation() {
}

void Simulation::init() {
    mState = Eigen::VectorXd::Zero(6);
}

void Simulation::control() {
}

void Simulation::step() {
    control();
    integrate();
}


void Simulation::integrate() {
}


} // namespace sim
} // namespace disneysimple



