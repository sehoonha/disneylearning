/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Manager.h"
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
}

Manager::~Manager() {
}

void Manager::init() {
    FOREACH(const utils::OptionItem& o, utils::Option::readAll("simulation.sim")) {
        const std::string type = o.attrString("type");
        if (type == SIMTYPE_BOX2D) {
            add( (new SimBox2D())->init() );
        } else if (type == SIMTYPE_MATHCALBONGO) {
            add( (new SimMathcalBongo())->init() );
        } else if (type == SIMTYPE_GAUSSIANPROCESS) {
            add( (new SimGaussianProcess())->init() );
        } else {
            LOG(FATAL) << "Invalid simulation type: " << type << endl;
        }
    }
    // add( (new SimMathcalBongo())->init() );
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Manager::load(const char* const filename) {
}

Simulator* Manager::availableSimulator(const char* const _type) {
    std::string querytype(_type);
    FOREACH(Simulator* sim, allSimulators()) {
        if (sim->type() ==  querytype) {
            return sim;
        }
    }
    return NULL;
    // return simulator(0);
}


// class Manager ends
////////////////////////////////////////////////////////////



} // namespace simulation
} // namespace disney


