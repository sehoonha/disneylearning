/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Simulator.h"
#include <fstream>
#include <iomanip>
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
    , MEMBER_INIT_NULL(policy)
    , MEMBER_INIT_NULL(eval)
{

}

Simulator::~Simulator() {
}

Simulator* Simulator::init() {
    return this;
}

int Simulator::controlStep() const {
    if (policy()) {
        return policy()->controlStep();
    }
    return 1;
}

void Simulator::step() {
    // LOG(INFO) << "==== step ====";
    if ( (controlStep() < 2) ||
         (numHistories() % controlStep()) == 1
          ) {
        control();
    }
    if (eval() == NULL || eval()->isFailed() == false ) {
        integrate();
    }
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

void Simulator::pushHistory() {
    SimulatorHistory sh;
    sh.fullstate = fullState();
    sh.state = state();
    sh.torque = mTorque;
    sh.cost = (eval() != NULL) ? eval()->cost() : 0.0;
    sh.contacts = mContacts;
    mHistory.push_back(sh);
}

void Simulator::updateToHistory(int index) {
    SimulatorHistory& sh = mHistory[index];
    setFullState(sh.fullstate);
    mTorque = sh.torque;
    if (eval()) {
        eval()->setCost(sh.cost);
    }
    mContacts = sh.contacts;
}

void Simulator::loadHistoryFromFile(const char* const filename) {
    clearHistory();
    
    std::ifstream fin(filename);
    if (fin.is_open() == false) {
        LOG(FATAL) << "cannot open " << filename;
        return;
    }

    int n = numDimState();
    int m = numDimTorque();
    LOG(INFO) << FUNCTION_NAME() << "type = " << this->type() << " : " << n << ", " << m;

    Eigen::VectorXd currState;
    Eigen::VectorXd currTorque;
    int loop = 0;
    for (; ; loop++) {
        currState  = Eigen::VectorXd::Zero(n);
        currTorque = Eigen::VectorXd::Zero(m);

        for (int i = 0; i < n; i++) {
            fin >> currState(i);
        }
        for (int i = 0; i < m; i++) {
            fin >> currTorque(i);
        }
        if (fin.fail()) {
            LOG(INFO) << "end of data at loop = " << loop;
            break;
        }
        setState(currState);
        mTorque = currTorque;
        pushHistory();
        // states.push_back(currState);
        // torques.push_back(currTorque);
    }

    fin.close();
}

void Simulator::renderInfo() {
    glColor3d(0.0, 0.0, 0.0);
    // utils::renderString(0.1, 0.1, "HAHAHA");
    std::stringstream sout;
    sout << std::setprecision(4) << std::fixed;
    sout << "[" << type() << "] ";
    utils::renderString(-0.3, 1.8, sout.str().c_str());

    sout.str("");
    if (eval()) {
        sout << "cost: " << eval()->cost();
    } else {
        sout << "no cost";
    }
    sout << " at " << time();
    utils::renderString(-0.5, 1.7, sout.str().c_str());

    int n = numDimConfig();
    int m = 3; // To show
    sout.str("");
    sout << "q: ";
    for (int i = 0; i < m; i++) sout << state()(i) << " ";
    utils::renderString(-0.5, 1.6, sout.str().c_str());

    sout.str("");
    sout << "dq: ";
    for (int i = 0; i < m; i++) sout << state()(i + n) << " ";
    utils::renderString(-0.5, 1.5, sout.str().c_str());


    sout.str("");
    sout << "Torque: " << mTorque(0);
    utils::renderString(-0.5, 1.4, sout.str().c_str());

    
}

// class Simulator ends
////////////////////////////////////////////////////////////

} // namespace simulation
} // namespace disney


