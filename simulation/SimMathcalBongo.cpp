/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "SimMathcalBongo.h"
#include "utils/CppCommon.h"

namespace disney {
namespace simulation {

////////////////////////////////////////////////////////////
// class SimMathcalBongo implementation

SimMathcalBongo::SimMathcalBongo()
    : Simulator(SIMTYPE_MATHCALBONGO)
{
}

SimMathcalBongo::~SimMathcalBongo() {
}

Simulator* SimMathcalBongo::init() {
    int n = numDimConfig();
    int m = numDimState();
    
    Eigen::VectorXd xEq(m);
    xEq << 0, 0, 0, 0, PI_2, -PI_2        , 0, 0, 0, 0, 0, 0;

    Eigen::VectorXd xOffset(m);
    double angIni = 2;
    xOffset << 0.0, (angIni * PI / 180), 0, 0, 0, 0        , 0, 0, 0, 0, 0, 0;
    // xOffset << 0.2, 0.3, 0.2, 0.2, -0.2, -0.2
    //     , 0, 0, 0, 0, 0, 0;
        
    mState = xEq + xOffset;

    mTorque = Eigen::VectorXd::Zero( numDimTorque() );

    setState(mState);
    clearHistory();
    LOG(INFO) << FUNCTION_NAME() << " OK";
    return this;
}

void SimMathcalBongo::integrate() {
    double h = mTimestep;
    double h_2 = 0.5 * h;

    Eigen::VectorXd x = mState;
    Eigen::VectorXd u = mTorque;

    // LOG(INFO) << "math.x = " << x.transpose();
    // LOG(INFO) << "math.u = " << u.transpose();

    Eigen::VectorXd k1 = deriv(x, u);
    Eigen::VectorXd k2 = deriv(x + h_2 * k1, u);
    Eigen::VectorXd k3 = deriv(x + h_2 * k2, u);
    Eigen::VectorXd k4 = deriv(x + h * k3, u);

    Eigen::VectorXd dx = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
    // LOG(INFO) << "dx = " << dx.transpose();

    mState = x + h * dx;
    for (int i = 0; i < mState.size(); i++) {
        double x = mState(i);
        if (x < -2.0 * PI) x = -2.0 * PI;
        if (x >  2.0 * PI) x =  2.0 * PI;
        mState(i) = x;
    }
    // mHistory.push_back(mState);

    // // Hard coded constraint..
    mState(3) = mState(2);
    mState(4) =  0.5 * PI - mState(2);
    mState(5) = -0.5 * PI - mState(2);
}

// class SimMathcalBongo ends
////////////////////////////////////////////////////////////


} // namespace simulation
} // namespace disney


