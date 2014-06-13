/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef LEARNING_LEARNINGPOLICYCMASEARCH_H
#define LEARNING_LEARNINGPOLICYCMASEARCH_H

#include "LearningAlgorithm.h"

namespace disney {
namespace learning {

class LearningPolicyCMASearch : public LearningAlgorithm {
public:
    LearningPolicyCMASearch();
    virtual ~LearningPolicyCMASearch();

    virtual void init();

    virtual void train(learning::Policy* _policy,
                       simulation::Simulator* _sim);
protected:
}; // class LearningPolicyCMASearch

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_LEARNINGPOLICYCMASEARCH_H


