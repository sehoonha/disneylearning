#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Dense>
#include <vector>
#include "hppcommon.h"

namespace disneysimple {
namespace sim {

/*
  q: config 
  dq: velocity
  x: state(=config and velocity)
  n: # dims of config
  m: # dims of state
 */
class Simulation {
public:
    Simulation();
    virtual ~Simulation();

    // State and torque manipulations
    int nDimConfig() const { return mState.size() / 2; }
    int nDimState() const { return mState.size(); }
    void setState(const Eigen::VectorXd& state ) { mState = state; }
    Eigen::VectorXd getState() const { return mState; }
    void setTorque(const Eigen::VectorXd& torque) { mTorque = torque; }
    Eigen::VectorXd getTorque() const { return mTorque; }

    // Simulation
    void init();
    void control();
    void step();

    // System and integration
    Eigen::VectorXd deriv(const Eigen::VectorXd& state, const Eigen::VectorXd& control);
    void integrate();

    // Render
    void render();

    // History
    int nStateHistory() { return mStateHistory.size(); }
    void updateToHistory(int index) { mState = mStateHistory[index]; }
    void updateToLatestHistory() { mState = mStateHistory[ nStateHistory() - 1]; }
protected:
    Eigen::VectorXd mState;
    Eigen::VectorXd mTorque;
    std::vector<Eigen::VectorXd> mStateHistory;
};

} // namespace sim
} // namespace disneysimple

#endif // #ifndef SIMULATION_H
