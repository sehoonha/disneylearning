/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef SIMULATION_SIMBOX2D_H
#define SIMULATION_SIMBOX2D_H

#include "utils/HppCommon.h"
#include "Simulator.h"

#define SIMTYPE_BOX2D "SimBox2D"

namespace disney {
namespace simulation {

struct SimBox2DImp; // Hide the implementation from the header

class SimBox2D : public Simulator {
public:
    SimBox2D();
    virtual ~SimBox2D();
    virtual Simulator* init();

    // State functions
    virtual int numDimState() const { return 6 * 2; }
    virtual void setState(const Eigen::VectorXd& _state);
    virtual Eigen::VectorXd state() const;
    virtual int numDimTorque() const { return 4; }

    // Full State functions -- for maximal simulators. In default, state == full state
    virtual void setFullState(const Eigen::VectorXd& _fullState);
    virtual Eigen::VectorXd fullState() const;


    // Simulation functions
    virtual void integrate();
    void applyTorque();
    virtual void reset();
    virtual void control();

    // Visualization functions
    virtual void render();

    double noiseTorqueLo() const { return mNoiseTorqueLo; }
    double noiseTorqueHi() const { return mNoiseTorqueHi; }
    void setNoiseTorqueLo(double _n) { mNoiseTorqueLo = _n; }
    void setNoiseTorqueHi(double _n) { mNoiseTorqueHi = _n; }
    double noiseSensorLo() const { return mNoiseSensorLo; }
    double noiseSensorHi() const { return mNoiseSensorHi; }
    void setNoiseSensorLo(double _n) { mNoiseSensorLo = _n; }
    void setNoiseSensorHi(double _n) { mNoiseSensorHi = _n; }
protected:
    SimBox2DImp* imp;
    double mNoiseTorqueLo;
    double mNoiseTorqueHi;
    double mNoiseSensorLo;
    double mNoiseSensorHi;

}; // class SimBox2D

} // namespace simulation
} // namespace disney

#endif // #ifndef SIMULATION_SIMBOX2D_H

