/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Simulator.h"
#include "utils/CppCommon.h"
#include "utils/LoadOpengl.h"
#include "utils/GLObjects.h"
#include "learning/Policy.h"
#include "Evaluator.h"

namespace disney {
namespace simulation {

////////////////////////////////////////////////////////////
// class Simulator implementation
Simulator::Simulator()
    : MEMBER_INIT_NULL(policy)
    , MEMBER_INIT_NULL(eval)
{
}

Simulator::Simulator(const char* const _type) 
    : mType(_type)
    , mTimestep(0.001)
    , mControlStep(2)
    , MEMBER_INIT_NULL(policy)
    , MEMBER_INIT_NULL(eval)
{

}

Simulator::~Simulator() {
}

Simulator* Simulator::init() {
    return this;
}

void Simulator::step() {
    
    if ( (mControlStep < 2) ||
         (numHistories() % mControlStep) == 1
          ) {
        control();
    }
    integrate();
    if (eval()) {
        eval()->eval(this);
    }
    
    pushHistory();
}

void Simulator::control() {
    Eigen::VectorXd x = state();
    mTorque = policy()->control(x);
}

void Simulator::integrate() {
}

void Simulator::reset() {
    updateToHistory(0);
    clearHistory();
    if (eval()) {
        eval()->reset();
    }
}

void Simulator::renderInfo() {
    glColor3d(0.0, 0.0, 0.0);
    // utils::renderString(0.1, 0.1, "HAHAHA");
    std::stringstream sout;
    sout << "[" << type() << "] ";
    sout << " at " << time();
    utils::renderString(-0.3, 1.8, sout.str().c_str());

    sout.str("");
    if (eval()) {
        sout << "cost: " << eval()->cost();
    } else {
        sout << "no cost";
    }
    utils::renderString(-0.3, 1.7, sout.str().c_str());
    
}

// class Simulator ends
////////////////////////////////////////////////////////////

} // namespace simulation
} // namespace disney


