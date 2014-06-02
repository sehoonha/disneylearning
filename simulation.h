#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Dense>
#include <vector>
#include "hppcommon.h"

namespace disneysimple {
namespace sim {

class Simulation {
public:
    Simulation();
    virtual ~Simulation();

    // State and torque manipulations
    int dim() const { return mState.size(); }
    void setState(const Eigen::VectorXd& state ) { mState = state; }
    Eigen::VectorXd getState() const { return mState; }
    void setTorque(const Eigen::VectorXd& torque) { mTorque = torque; }
    Eigen::VectorXd getTorque() const { return mTorque; }

    // Simulation
    void init();
    void control();
    void step();

    Eigen::VectorXd deriv(const Eigen::VectorXd& state, const Eigen::VectorXd& control);
    void integrate();

protected:
    Eigen::VectorXd mState;
    Eigen::VectorXd mTorque;
    std::vector<Eigen::VectorXd> mStateHistory;
};

} // namespace sim
} // namespace disneysimple

#endif // #ifndef SIMULATION_H
