/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef SIMULATION_EVALUATOR_H
#define SIMULATION_EVALUATOR_H

#include "utils/HppCommon.h"

namespace disney {
namespace simulation {
class Simulator;
} // namespace simulation
} // namespace disney


namespace disney {
namespace simulation {

class Evaluator {
public:
    Evaluator();
    virtual ~Evaluator();

    virtual double eval(Simulator* _sim) = 0;
protected:
}; // class Evaluator

} // namespace simulation
} // namespace disney

#endif // #ifndef SIMULATION_EVALUATOR_H

