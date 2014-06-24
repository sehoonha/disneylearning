/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef SIMULATION_SIMGAUSSIANPROCESS_H
#define SIMULATION_SIMGAUSSIANPROCESS_H

#include "utils/HppCommon.h"
#include "Simulator.h"

#define SIMTYPE_GAUSSIANPROCESS "SimGaussianProcess"

namespace disney {
namespace simulation {
class SimMathcalBongo;
} // namespace simulation
namespace learning {
class GaussianProcess;
} // namespace learning
} // namespace disney

namespace disney {
namespace simulation {

class SimGaussianProcess : public Simulator {
public:
    SimGaussianProcess();
    virtual ~SimGaussianProcess();
    virtual Simulator* init();

    // Training functions
    void train();
    void optimize();

    // State functions
    virtual int numDimState() const { return 6 * 2; }
    virtual void setState(const Eigen::VectorXd& _state) { mState = _state; }
    virtual Eigen::VectorXd state() const { return mState; }

    virtual int numDimTorque() const { return 4; }
    
    // Simulation functions
    virtual void integrate();
    Eigen::VectorXd deriv(const Eigen::VectorXd& state, const Eigen::VectorXd& control);

    // 
    virtual void updateToHistory(int index);

    // Visualization functions
    virtual void render();

protected:
    Eigen::VectorXd mState;

    SimMathcalBongo* model;
    learning::GaussianProcess* gp;
}; // class SimGaussianProcess

} // namespace simulation
} // namespace disney

#endif // #ifndef SIMULATION_SIMGAUSSIANPROCESS_H

