/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "PolicyFeedback.h"
#include "utils/CppCommon.h"

namespace disney {
namespace learning {

////////////////////////////////////////////////////////////
// class PolicyFeedback implementation

PolicyFeedback::PolicyFeedback()
    : Policy(5)
{
}

PolicyFeedback::~PolicyFeedback() {
}

Policy* PolicyFeedback::init() {
    int n = 6;
    // Parameters
    double radius = 0.05;
    double rw     = radius;
    double lrl1 = 1;
    double lrl2 = 0.1;
    double lll1 = 1;
    double lll2 = 0.1;

    Eigen::MatrixXd C(5, 2 * n);
    C << - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,  
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,    
        1, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0;  
    this->C = C;

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(4, 5);
    this->F = F;
    this->K = F * C;

    Eigen::VectorXd init(numDimParams());
    // init << -2716691.61124318, -1189377.26019541, 953603.332319511, 10071.8805575885, 768507.689769501;
    // init << 18324,-33046,12511.1,16938,25786.9;

    // init << 9400.85,-19467.2,9782.43,13117.1,20555.5; // Trained in Box 2D
    init << -719.813, -45.9087, -541.661, 0980.184, -1958.46; // Trained in Math + GP
    setParams(init);
    LOG(INFO) << FUNCTION_NAME() << " OK";
    return this;
}

void PolicyFeedback::setParams(const Eigen::VectorXd& _params) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 5; j++) {
            if (i == 0 || i == 1) {
                F(i, j) = _params(j);
            } else {
                F(i, j) = -_params(j);
            }

        }
    }
    this->K = F * C;
}

Eigen::VectorXd PolicyFeedback::control(const Eigen::VectorXd& _state) {
    // Fetch the state
    const Eigen::VectorXd& x = _state;
    int n = x.size() / 2;
    // Parameters
    double maxTorq = 20.0;

    // Equilibrium state
    Eigen::VectorXd qEq(n);
    qEq << 0.0, 0.0, 0.0, 0.0, PI/2.0, -PI/2.0;
    Eigen::VectorXd dqEq(n);
    dqEq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd xEq(n * 2);
    xEq.head(n) = qEq;
    xEq.tail(n) = dqEq;
        
    // Calculate u
    Eigen::VectorXd u = -K * (x - xEq);
    for(int i = 0; i < u.size(); i++) {
        if (fabs(u(i)) > maxTorq) {
            if (u(i) > 0) {
                u(i) = maxTorq;
            } else {
                u(i) = -maxTorq;
            }
        }
    }

    u = (Eigen::VectorXd(4) << 0.0, 0.0, 0.0, 0.0).finished();
    // u = (Eigen::VectorXd(4) << -20.0, -20.0, 20.0, 20.0).finished();
    // u = (Eigen::VectorXd(4) << -20.0, -20.0, 20.0, 20.0).finished();


    // // Calculate u -- pose tracker
    // Eigen::VectorXd dq_bar = -K * (x - xEq);
    // Eigen::VectorXd qdot = _state.tail(4);
    // double KS = 20.0;
    // double KD = sqrt(KS);
    // // -KS * ( q - qbar) - KD * xdot (where qbar = q + dq_bar) 
    // Eigen::VectorXd u = -KS * (-dq_bar) - KD * qdot;

    // for(int i = 0; i < u.size(); i++) {
    //     if (fabs(u(i)) > maxTorq) {
    //         if (u(i) > 0) {
    //             u(i) = maxTorq;
    //         } else {
    //             u(i) = -maxTorq;
    //         }
    //     }
    // }
    return u;
}

// class PolicyFeedback ends
////////////////////////////////////////////////////////////


} // namespace learning
} // namespace disney


