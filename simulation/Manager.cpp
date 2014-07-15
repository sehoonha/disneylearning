/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Manager.h"
#include <boost/thread/mutex.hpp>
#include "utils/CppCommon.h"
#include "utils/Option.h"
#include "Simulator.h"
#include "SimMathcalBongo.h"
#include "SimBox2D.h"
#include "SimGaussianProcess.h"

namespace disney {
namespace simulation {

////////////////////////////////////////////////////////////
// class Manager implementation
Manager::Manager() {
    mMutex = new boost::mutex();
}

Manager::~Manager() {
    delete mMutex;
}

void Manager::init() {
    FOREACH(const utils::OptionItem& o, utils::Option::readAll("simulation.sim")) {
        const std::string type = o.attrString("type");
        bool isReserved = (o.hasAttr("isReserved") && o.attrBool("isReserved"));
        if (type == SIMTYPE_BOX2D) {
            add( (new SimBox2D())->init(), isReserved );
        } else if (type == SIMTYPE_MATHCALBONGO) {
            add( (new SimMathcalBongo())->init(), isReserved );
        } else if (type == SIMTYPE_GAUSSIANPROCESS) {
            add( (new SimGaussianProcess())->init(), isReserved );
        } else {
            LOG(FATAL) << "Invalid simulation type: " << type << endl;
        }
    }
    // add( (new SimMathcalBongo())->init() );

    // LOG(INFO) << "# of simulators: " << mSimulators.size();
    // LOG(INFO) << "# of reserved simulators: " << mReservedSimulators.size();
    // for (int i = 0; i < 6; i++) {
    //     Simulator* s = unoccupiedSimulator(SIMTYPE_GAUSSIANPROCESS);
    //     LOG(INFO) << "occupy " << s->type() << " " << s->id();
    //     if (i % 3 == 0) {
    //         LOG(INFO) << "release " << s->type() << " " << s->id();
    //         markSimulatorAsUnoccupied(s);
    //     }
    // }
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Manager::load(const char* const filename) {
}

Simulator* Manager::findSimulator(const char* const _type) {
    std::string querytype(_type);
    FOREACH(Simulator* sim, mSimulators) {
        if (sim->type() == querytype) {
            return sim;
        }
    }
    return NULL;
}

Simulator* Manager::unoccupiedSimulator(const char* const _type) {
    boost::mutex::scoped_lock lock(*mMutex);

    std::string querytype(_type);
    FOREACH(Simulator* sim, mSimulators) {
        if (sim->type() == querytype && sim->isOccupied() == false) {
            sim->setIsOccupied(true);
            return sim;
        }
    }
    FOREACH(Simulator* sim, mReservedSimulators) {
        if (sim->type() == querytype && sim->isOccupied() == false) {
            sim->setIsOccupied(true);
            return sim;
        }
    }

    return NULL;
    // return simulator(0);
}

bool Manager::markSimulatorAsUnoccupied(Simulator* _sim) {
    boost::mutex::scoped_lock lock(*mMutex);
    _sim->setIsOccupied(false);
}


Simulator* Manager::simulator(int index) {
    int n = mSimulators.size();
    if (index < n) {
        return mSimulators[index];
    } else {
        return mReservedSimulators[index - n];
    }
    // return mSimulators[index];
}

void Manager::add(Simulator* _sim, bool isReserved) {
    _sim->setId( mSimulators.size() + mReservedSimulators.size() );
    _sim->setIsReserved(isReserved);
    if (!isReserved) {
        mSimulators.push_back(_sim);
    } else {
        mReservedSimulators.push_back(_sim);
    }
}

std::vector<Simulator*> Manager::allExistingSimulators() {
    std::vector<Simulator*> ret;
    FOREACH(Simulator* sim, mSimulators) {
        ret.push_back(sim);
    }
    FOREACH(Simulator* sim, mReservedSimulators) {
        ret.push_back(sim);
    }
    return ret;
}

// class Manager ends
////////////////////////////////////////////////////////////



} // namespace simulation
} // namespace disney


