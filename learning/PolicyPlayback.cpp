/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "PolicyPlayback.h"
#include <fstream>
#include "utils/CppCommon.h"

namespace disney {
namespace learning {

////////////////////////////////////////////////////////////
// class PolicyPlayback implementation
PolicyPlayback::PolicyPlayback()
    : Policy(0)
{
}

PolicyPlayback::~PolicyPlayback() {
}

Policy* PolicyPlayback::init() {
    reset();
    return this;
}

Policy* PolicyPlayback::duplicate() {
    PolicyPlayback* ret = new PolicyPlayback();
    ret->mName = this->mName;
    ret->mDim = this->mDim;
    ret->mParams = this->mParams;
    ret->mControlStep = this->mControlStep;
    ret->mIndex = this->mIndex;
    ret->mTorques = this->mTorques;
    return ret;
}

void PolicyPlayback::setParams(const Eigen::VectorXd& _params) {
    Policy::setParams(_params);
}

bool PolicyPlayback::load(const char* const filename, int n, int m) {
    std::ifstream fin(filename);
    if (fin.is_open() == false) {
        return false;
    }

    for (int loop = 0; ; loop++) {
        Eigen::VectorXd currState  = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd currTorque = Eigen::VectorXd::Zero(m);

        for (int i = 0; i < n; i++) {
            fin >> currState(i);
        }
        for (int i = 0; i < m; i++) {
            fin >> currTorque(i);
        }
        if (fin.fail()) {
            LOG(INFO) << "end of data at loop = " << loop;
            break;
        }
        mTorques.push_back( currTorque );

    }
    LOG(INFO) << FUNCTION_NAME() << " OK. mTorques.size() = " << mTorques.size();
    return true;
    
}

Eigen::VectorXd PolicyPlayback::control(const Eigen::VectorXd& _state) {
    if (mIndex < mTorques.size()) {
        Eigen::VectorXd ret = mTorques[mIndex];
        LOG(INFO) << mIndex << " : " << ret.transpose();
        return ret;
    }
    LOG(WARNING) << "we do not have recorded torque data anymore...";
    return Eigen::VectorXd::Zero( mTorques[0].size() );
}

// class PolicyPlayback ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disney


