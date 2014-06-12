/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef LEARNING_LEARNINGALGORITHM_H
#define LEARNING_LEARNINGALGORITHM_H

namespace disney {

namespace simulation {
class Simulator;
class Evaluator;
} // namespace simulation

namespace learning {
class Policy;
} // namespace learning

} // namespace disney


namespace disney {
namespace learning {

class LearningAlgorithm {
public:
    LearningAlgorithm();
    virtual ~LearningAlgorithm();

    virtual void train(learning::Policy* _policy,
                       simulation::Simulator* _sim,
                       simulation::Evaluator* _eval) = 0;
protected:
}; // class LearningAlgorithm

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_LEARNINGALGORITHM_H

