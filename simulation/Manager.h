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

namespace boost {
class mutex;
} // namespace boost

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

    // Manage simulators
    int numSimulators() const { return mSimulators.size(); }
    Simulator* simulator(int index);
    void add(Simulator* _sim, bool isReserved = false);

    Simulator* findSimulator(const char* const _type);
    Simulator* unoccupiedSimulator(const char* const _type);
    bool markSimulatorAsUnoccupied(Simulator* _sim);
    std::vector<Simulator*>& allSimulators() { return mSimulators; }
    std::vector<Simulator*>& allReservedSimulators() { return mReservedSimulators; }
    std::vector<Simulator*> allExistingSimulators();
protected:
    std::vector<Simulator*> mSimulators;
    std::vector<Simulator*> mReservedSimulators;
    boost::mutex* mMutex;

}; // class Manager

} // namespace simulation
} // namespace disney

#endif // #ifndef SIMULATION_MANAGER_H

