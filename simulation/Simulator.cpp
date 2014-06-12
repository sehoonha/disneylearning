/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Simulator.h"
#include "utils/CppCommon.h"

namespace disney {
namespace simulation {

////////////////////////////////////////////////////////////
// class Simulator implementation
Simulator::Simulator() {
}

Simulator::~Simulator() {
}

void Simulator::init() {
}

void Simulator::step() {
    
    if ( (numHistories() % mNumControlStep) == 1 ||
         (mNumControlStep < 2) ) {
        control();
    }
    integrate();
}

void Simulator::control() {
}

void Simulator::integrate() {
}

// class Simulator ends
////////////////////////////////////////////////////////////

} // namespace simulation
} // namespace disney


