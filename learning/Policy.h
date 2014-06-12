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
    Policy();
    virtual ~Policy();

    virtual void setParams(const Eigen::VectorXd& _params) = 0;
    virtual Eigen::VectorXd params() const = 0;

    virtual Eigen::VectorXd control(const Eigen::VectorXd& _state) = 0;

protected:
}; // class Policy

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_POLICY_H

