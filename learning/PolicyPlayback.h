/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef LEARNING_POLICYPLAYBACK_H
#define LEARNING_POLICYPLAYBACK_H

#include <vector>
#include "Policy.h"

namespace disney {
namespace learning {

class PolicyPlayback : public Policy {
public:
    PolicyPlayback();
    virtual ~PolicyPlayback();

    virtual Policy* init();
    virtual Policy* duplicate();
    virtual void setParams(const Eigen::VectorXd& _params);

    virtual void reset() { mIndex = 1; }
    virtual void step() { mIndex++; }

    bool load(const char* const filename, int n, int m);
    virtual Eigen::VectorXd control(const Eigen::VectorXd& _state);
    
protected:
    int mIndex;
    std::vector<Eigen::VectorXd> mTorques;
}; // class PolicyPlayback

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_POLICYPLAYBACK_H

