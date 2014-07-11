/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Policy.h"
#include "utils/CppCommon.h"

namespace disney {
namespace learning {

////////////////////////////////////////////////////////////
// class Policy implementation
Policy::Policy(int _dim)
    : mDim(_dim)
    , mControlStep(2)
{
}

Policy::~Policy() {
}

Policy* Policy::init() {
    return this;
}

// class Policy ends
////////////////////////////////////////////////////////////


} // namespace learning
} // namespace disney


