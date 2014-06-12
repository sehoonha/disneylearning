/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef SIMULATION_MANAGER_H
#define SIMULATION_MANAGER_H

#include <vector>
#include "utils/HppCommon.h"

namespace disney {
namespace simulation {
class Simulator;
} // namespace simulation
} // namespace disney

namespace disney {
namespace simulation {

class Manager {
public:
    // Constructor and destructor 
    Manager();
    virtual ~Manager();

    // Initialization functions
    void init();
    void load(const char* const filename);

    // Fetch simulators
    int numSimulators() const { return mSimulators.size(); }
    Simulator* simulator(int index) { return mSimulators[index]; }
    Simulator* availableSimulator(const char* const _type);
protected:
    std::vector<Simulator*> mSimulators;
}; // class Manager

} // namespace simulation
} // namespace disney

#endif // #ifndef SIMULATION_MANAGER_H

