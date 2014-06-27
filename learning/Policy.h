/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef LEARNING_POLICY_H
#define LEARNING_POLICY_H

#include <Eigen/Dense>
#include "utils/HppCommon.h"

namespace disney {
namespace learning {

class Policy {
public:
    Policy(int _dim);
    virtual ~Policy();
    virtual Policy* init();

    virtual int numDimParams() { return mDim; }
    virtual void setParams(const Eigen::VectorXd& _params) { mParams = _params; }
    virtual Eigen::VectorXd params() const { return mParams; }

    virtual void reset() {}
    virtual void step() {}
    
    virtual Eigen::VectorXd control(const Eigen::VectorXd& _state) = 0;

protected:
    int mDim;
    Eigen::VectorXd mParams;
}; // class Policy

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_POLICY_H

