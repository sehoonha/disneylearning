/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Evaluator.h"
#include "utils/CppCommon.h"
#include "utils/Option.h"
#include "Simulator.h"

namespace disney {
namespace simulation {

////////////////////////////////////////////////////////////
// class Evaluator implementation
Evaluator::Evaluator() {
    reset();
    this->maxSimLoop   = utils::Option::read("simulation.eval.maxSimLoop").toInt();
}

Evaluator::~Evaluator() {
}

void Evaluator::reset() {
    mCost = 0.0;
    mIsFailed = false;
}

double Evaluator::eval(Simulator* _sim) {
    return eval2(_sim);
}

double Evaluator::eval2(Simulator* _sim) {

    // Fetch the state
    int n = _sim->numDimConfig();
    const Eigen::VectorXd& x = _sim->state();
    const Eigen::VectorXd& u = _sim->torque();

    // Equilibrium state
    Eigen::VectorXd qEq(n);
    qEq << 0.0, 0.0, 0.0, 0.0, PI/2.0, -PI/2.0;
    Eigen::VectorXd dqEq(n);
    dqEq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd xEq(n * 2);
    xEq.head(n) = qEq;
    xEq.tail(n) = dqEq;

    // Define R matrix
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n * 2, n * 2);
    R(0, 0) = 1.0;
    R(1, 1) = 1.0;
    R(2, 2) = R(3, 3) = R(4, 4) = R(5, 5) = 1.0;
    R(6, 6) = 0.1;
    R(7, 7) = 0.1;
    R(8, 8) = R(9, 9) = R(10, 10) = R(11, 11) = 0.1;

    R *= 0.01;


    // Add a cost
    Eigen::VectorXd dx = (x - xEq);
    double costNow = dx.dot( R * dx );

    // Add a cost for torque
    {
        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(u.size(), u.size());
        // T(0, 0) = 0.0;
        T(0, 0) = 0.00001;
        costNow += u.dot(T * u);

        Eigen::MatrixXd U = Eigen::MatrixXd::Zero(u.size(), u.size());
        U(0, 0) = 0.0001;
        Eigen::VectorXd du = (u - _sim->lastTorque());
        costNow += du.dot(U * du);

    }

    mCost += costNow;

    // Check failure
    const double FAILED_COST = 1000.0;

    // if (fabs(x(0)) > 0.7 ||
    //     fabs(x(1)) > 0.7 ||
    //     fabs(x(0) + x(1)) > 0.5 ||
    //     fabs(x(2)) > 0.7) {
    if (fabs(x(0) + x(1) + x(2)) > 0.5  // When the robot is tilted too much
        || fabs(x(0) + x(1)) > 0.3 // When the board hit the ground
        || fabs(x(6) + x(7)) > 6.2 // When the body of robot moves too fast
        || fabs(x(6)) > 6.2 // Velocity of the wheel reaches the limit (2 PI)
        || fabs(x(7)) > 6.2 // Velocity of the board reaches the limit (2 PI)
        ) {
        mIsFailed = true;
    }
    
    if (mIsFailed) {
        mCost += FAILED_COST;
    }

    bool isFinal = (_sim->numHistories() == this->maxSimLoop);
    if (isFinal) {
        Eigen::MatrixXd RF = Eigen::MatrixXd::Zero(n * 2, n * 2);
        RF(0, 0) = 1.0;
        RF(1, 1) = 1.0;
        RF(2, 2) = RF(3, 3) = RF(4, 4) = RF(5, 5) = 1.0;
        RF(6, 6) = 0.5;
        RF(7, 7) = 0.5;
        RF(8, 8) = RF(9, 9) = RF(10, 10) = RF(11, 11) = 0.5;
        RF *= 2.0;
        double finalCost = dx.dot(RF * dx);
        mCost += finalCost;
        // LOG(INFO) << _sim->type() << " reaches the final!! : " << finalCost;
    }
    
    return mCost;
}


double Evaluator::eval1(Simulator* _sim) {
    const double COST_FAIL = 1000.0;
    if (isFailed()) {
        mCost += COST_FAIL;
        return mCost;
    }
    
    // Parameters
    double offset = 0;
    double radius = 0.05;
    double rw     = radius;
    double lb     = 0.6;

    double lrl1 = 1;
    double lrl2 = 0.1;
    double lll1 = 1;
    double lll2 = 0.1;

    // Fetch the state
    int n = _sim->numDimConfig();
    const Eigen::VectorXd& x = _sim->state();
    const Eigen::VectorXd& u = _sim->torque();

    Eigen::VectorXd q = x.head(n);
    double alphaw  = q(0);
    double alphab  = q(1);
    double al      = 0.0;
    double ar      = 0.0;
    double thetal1 = q(2);
    double thetar1 = q(3);
    double thetal2 = q(4);
    double thetar2 = q(5);


    Eigen::Vector3d top;
    top << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - lrl2*sin(alphab + alphaw + thetar1 + thetar2) - rw*sin(alphab + alphaw) - alphaw*rw - lrl1*sin(alphab + alphaw + thetar1),
        rw + lrl2*cos(alphab + alphaw + thetar1 + thetar2) + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw) + lrl1*cos(alphab + alphaw + thetar1)  ;

    // wheel position
    Eigen::Vector3d wheel;
    wheel << 0,
        offset-alphaw*rw,
        rw;
    // feet cart positions
    Eigen::Vector3d rightCart;
    rightCart << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - rw*sin(alphab + alphaw) - alphaw*rw,
        rw + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw);
    
    Eigen::Vector3d leftCart;
    leftCart << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw),
        rw + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw)  ;          
    Eigen::Vector3d cart = 0.5 * (leftCart + rightCart);

    Eigen::Vector3d boardRight;
    boardRight << 0,
        offset- alphaw*rw - cos(alphab + alphaw)*(lb/2 - alphab*rw) - rw*sin(alphab + alphaw),
        rw - sin(alphab + alphaw)*(lb/2 - alphab*rw) + rw*cos(alphab + alphaw);

    Eigen::Vector3d boardLeft;
    boardLeft << 0,
        offset+cos(alphab + alphaw)*(lb/2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw),
        rw + sin(alphab + alphaw)*(lb/2 + alphab*rw) + rw*cos(alphab + alphaw);     

    double currentCost = 0;

    // Define axes
    const int X = 1;
    const int Y = 2;

    // Check the fail
    bool failed = false;
    if (top(Y) < 0.2 || cart(Y) < 0.0) {
        failed = true;
    }
    const double BOX2D_TOUCH_HEIGHT = 0.014;
    if (boardLeft(Y) < BOX2D_TOUCH_HEIGHT || boardRight(Y) < BOX2D_TOUCH_HEIGHT) {
        failed = true;
    }
    if (wheel(Y) < 0.01 || fabs(wheel(X)) > 0.15) {
        failed = true;
    }
    if (failed) {
        currentCost += COST_FAIL;
    }

    
    // From now, ignore the Y offset
    top(Y) = wheel(Y) = cart(Y) = 0.0;

    double cost = (top - wheel).squaredNorm()
        + (top - cart).squaredNorm()
        + (wheel - cart).squaredNorm();
    currentCost += cost;


    Eigen::VectorXd dq = x.tail(n);
    dq = dq.head(3);
    mCost += 0.01 * dq.squaredNorm();

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(u.size(), u.size());
    R(0, 0) = 1e-6;
    currentCost += u.dot(R * u);

    // Final integrate
    double t = _sim->time();
    double coef = pow(0.1, t);
    mCost += (coef * currentCost);
    return mCost;
}

// class Evaluator ends
////////////////////////////////////////////////////////////



} // namespace simulation
} // namespace disney


