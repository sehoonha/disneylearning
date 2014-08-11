/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef LEARNING_LEARNINGGPSIMSEARCH_H
#define LEARNING_LEARNINGGPSIMSEARCH_H

#include "LearningAlgorithm.h"

namespace disney {
namespace simulation {
class Simulator;
class Manager;
} // namespace simulation
namespace learning {
class Policy;
} // namespace learning
} // namespace disney

namespace disney {
namespace learning {

struct LearningGPSimSearchImp;

class LearningGPSimSearch : public LearningAlgorithm {
public:
    LearningGPSimSearch();
    virtual ~LearningGPSimSearch();

    virtual void init();

    virtual void train(simulation::Manager* _manager,
                       learning::Policy* _policy,
                       simulation::Simulator* _sim0,
                       simulation::Simulator* _sim1,
                       bool launchThread = true);

    virtual void togglePause();

protected:
    LearningGPSimSearchImp* imp;
}; // class LearningGPSimSearch

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_LEARNINGGPSIMSEARCH_H

