/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "LearningAlgorithm.h"
#include "utils/CppCommon.h"

namespace disney {
namespace learning {

////////////////////////////////////////////////////////////
// class LearningAlgorithm implementation
LearningAlgorithm::LearningAlgorithm() {
}

LearningAlgorithm::~LearningAlgorithm() {
}

void LearningAlgorithm::init() {
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

// class LearningAlgorithm ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disney


