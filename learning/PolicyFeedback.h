/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef LEARNING_POLICYFEEDBACK_H
#define LEARNING_POLICYFEEDBACK_H

#include "Policy.h"

namespace disney {
namespace learning {

class PolicyFeedback : public Policy {
public:
    PolicyFeedback();
    virtual ~PolicyFeedback();

    virtual Policy* init();
    virtual Policy* duplicate();

    virtual void setParams(const Eigen::VectorXd& _params);

    virtual Eigen::VectorXd control(const Eigen::VectorXd& _state);

protected:
    Eigen::MatrixXd K;
    Eigen::MatrixXd F;
    Eigen::MatrixXd C;

}; // class PolicyFeedback

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_POLICYFEEDBACK_H

