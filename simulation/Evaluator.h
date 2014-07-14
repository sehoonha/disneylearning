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

    double cost() const { return mCost; }
    void setCost(double _cost) { mCost = _cost; }
    virtual void reset();
    virtual double eval(Simulator* _sim);
    bool isFailed() const { return mIsFailed; }
    double eval1(Simulator* _sim);
    double eval2(Simulator* _sim);
protected:
    double mCost;
    bool mIsFailed;
}; // class Evaluator

} // namespace simulation
} // namespace disney

#endif // #ifndef SIMULATION_EVALUATOR_H

