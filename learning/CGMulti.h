/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef LEARNING_CGMULTI_H
#define LEARNING_CGMULTI_H

#include <vector>
#include "gp.h"


namespace libgp
{

class CGMulti {
public:
    CGMulti();
    virtual ~CGMulti();
    void maximize(std::vector<GaussianProcess*>& _gp_array, size_t n=100, bool verbose=1);

    double log_likelihood();
    double log_likelihood2();
    Eigen::VectorXd log_likelihood_gradient();
    Eigen::VectorXd get_loghyper();
    void set_loghyper(const Eigen::VectorXd& x);
protected:
    std::vector<GaussianProcess*> gp_array;
}; // class CGMulti

}

#endif // #ifndef LEARNING_CGMULTI_H

