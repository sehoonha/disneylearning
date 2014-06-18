/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef SIMULATION_SIMULATOR_H
#define SIMULATION_SIMULATOR_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include "utils/HppCommon.h"

namespace disney {
namespace learning {
class Policy;
} // namespace learning
namespace simulation {
class Evaluator;
} // namespace simulation
} // namespace disney


namespace disney {
namespace simulation {

struct SimulatorHistory;
class Simulator;

struct SimulatorHistory {
private:
    friend class Simulator;
    Eigen::VectorXd fullstate;
    Eigen::VectorXd torque;
    double cost;
    Eigen::VectorXd contacts;
};

class Simulator {
public:
    // Constructor and destructor 
    Simulator();
    Simulator(const char* const _type);
    virtual ~Simulator();

    virtual Simulator* init();
    std::string type() const { return mType; }

    // State functions
    virtual int numDimConfig() const { return numDimState() / 2; }
    virtual int numDimState() const = 0;

    virtual void setState(const Eigen::VectorXd& _state) = 0;
    virtual Eigen::VectorXd state() const = 0;

    virtual int numDimTorque() const = 0;
    virtual void setTorque(const Eigen::VectorXd& _torque) { mTorque = _torque; }
    virtual Eigen::VectorXd torque() const { return mTorque; }
    
    virtual double time() const { return (timeStep() * (double)(numHistories() - 1)); }
    virtual double timeStep() const { return mTimestep; }

    // Full State functions -- for maximal simulators. In default, state == full state
    virtual void setFullState(const Eigen::VectorXd& _fullState) { setState(_fullState); }
    virtual Eigen::VectorXd fullState() const { return state(); }

    // Simulation functions
    virtual void step();
    virtual void control();
    virtual void integrate();
    virtual void reset();

    // History functions
    virtual void clearHistory() { mHistory.clear(); pushHistory(); }
    virtual int numHistories() const { return mHistory.size(); }
    // virtual void pushHistory() { mHistory.push_back( fullState() ); }
    // virtual void updateToHistory(int index) { setFullState( mHistory[index] ); }
    virtual void pushHistory();
    virtual void updateToHistory(int index);
    virtual void updateToLatestHistory() { updateToHistory( numHistories() - 1 ); }

    // Visualization functions
    virtual void renderInfo();
    virtual void render() = 0;
protected:
    std::string mType;
    double mTimestep;
    std::vector<SimulatorHistory> mHistory;
    // std::vector<Eigen::VectorXd> mHistory; // Record in the full state
    // -- removed -- Eigen::VectorXd mState // state is not always a single vector

    int mControlStep;
    MEMBER_PTR(learning::Policy*, policy);
    Eigen::VectorXd mTorque;
    Eigen::VectorXd mContacts;

    MEMBER_PTR(Evaluator*, eval);

}; // class Simulator

} // namespace simulation
} // namespace disney

#endif // #ifndef SIMULATION_SIMULATOR_H

