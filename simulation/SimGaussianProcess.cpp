/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "SimGaussianProcess.h"
#include "utils/CppCommon.h"
#include "SimMathcalBongo.h"
#include "learning/GaussianProcess.h"

namespace disney {
namespace simulation {

////////////////////////////////////////////////////////////
// class SimGaussianProcess implementation
SimGaussianProcess::SimGaussianProcess()
    : Simulator(SIMTYPE_GAUSSIANPROCESS)
{
}

SimGaussianProcess::~SimGaussianProcess() {
}

Simulator* SimGaussianProcess::init() {
    int n = numDimConfig();
    int m = numDimState();
    
    Eigen::VectorXd xEq(m);
    xEq << 0, 0, 0, 0, PI_2, -PI_2        , 0, 0, 0, 0, 0, 0;

    Eigen::VectorXd xOffset(m);
    double angIni = 0.3;
    xOffset << 0.0, (angIni * PI / 180), 0, 0, 0, 0        , 0, 0, 0, 0, 0, 0;
        
    mState = xEq + xOffset;

    mTorque = Eigen::VectorXd::Zero( numDimTorque() );

    setState(mState);
    clearHistory();

    // initialize two important classes
    model = (new SimMathcalBongo());
    model->init();
    gp = (new learning::GaussianProcess());

    LOG(INFO) << FUNCTION_NAME() << " OK";
    return this;
}


void SimGaussianProcess::train() {
    //
    // Questions:
    // - when this should be called?
    // - how can we train GP?
    // 
}

void SimGaussianProcess::integrate() {
    Eigen::VectorXd x_prev = mState;

    // 1. Generate delta from our siimulation model
    model->setState( mState );
    model->setTorque( mTorque );
    model->integrate();
    Eigen::VectorXd dx = model->state() - x_prev;

    // 2. Correct the dynamics
    Eigen::VectorXd input( 7 );
    input(0) = x_prev(0);
    input(1) = x_prev(1);
    input(2) = x_prev(2);
    input(3) = x_prev(6 + 0);
    input(4) = x_prev(6 + 1);
    input(5) = x_prev(6 + 2);
    input(6) = mTorque(0);

    Eigen::VectorXd dx_delta = Eigen::VectorXd::Zero( 12 );
    // Eigen::VectorXd output = gp->predict( input ); // output = [dq0, dq1, dq2, ddq0, ddq1, ddq2]'
    // for (int i = 0; i < 3; i++) {
    //     dx_delta(i) = output(i);
    //     dx_delta(i + 6) = output(i + 3);
    // }

    // 3. To the next state
    Eigen::VectorXd x_curr = mState + dx + dx_delta;
    mState = x_curr;

    // 4. Finalize the state by maintaining the constraints
    // // Hard coded constraint..
    mState(3) = mState(2);
    mState(4) =  0.5 * PI - mState(2);
    mState(5) = -0.5 * PI - mState(2);

    mState(9) = mState(8);
    mState(10) = -mState(8);
    mState(11) = -mState(8);

    model->setState( mState );
}

Eigen::VectorXd
SimGaussianProcess::deriv(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    return model->deriv(state, control);
}

void SimGaussianProcess::render() {
    model->render();
}


// class SimGaussianProcess ends
////////////////////////////////////////////////////////////



} // namespace simulation
} // namespace disney


