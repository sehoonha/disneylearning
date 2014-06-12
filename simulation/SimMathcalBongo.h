/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef SIMULATION_SIMMATHCALBONGO_H
#define SIMULATION_SIMMATHCALBONGO_H

#include "utils/HppCommon.h"
#include "Simulator.h"

#define SIMTYPE_MATHCALBONGO "SimMathcalBongo"

namespace disney {
namespace simulation {

class SimMathcalBongo : public Simulator {
public:
    // Constructor and destructor 
    SimMathcalBongo();
    virtual ~SimMathcalBongo();
    virtual Simulator* init();

    // State functions
    virtual int numDimState() const { return 6 * 2; }
    virtual void setState(const Eigen::VectorXd& _state) { mState = _state; }
    virtual Eigen::VectorXd state() const { return mState; }

    virtual int numDimTorque() const { return 4; }

    // Simulation functions
    virtual void integrate();
    Eigen::VectorXd deriv(const Eigen::VectorXd& state, const Eigen::VectorXd& control);

    // Visualization functions
    virtual void render();

    
protected:
    Eigen::VectorXd mState;
    
}; // class SimMathcalBongo

} // namespace simulation
} // namespace disney

#endif // #ifndef SIMULATION_SIMMATHCALBONGO_H

