/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef LEARNING_GAUSSIANPROCESS_H
#define LEARNING_GAUSSIANPROCESS_H

#include <Eigen/Dense>
#include "utils/HppCommon.h"

namespace disney {
namespace learning {

struct GaussianProcessImp;

class GaussianProcess {
public:
    GaussianProcess();
    virtual ~GaussianProcess();

    void init();
    void createModel(const Eigen::MatrixXd& _X, const Eigen::MatrixXd& _Y);
    void loadModel(const char* const filename);
    void saveModel(const char* const filename);

    void setTrainingData(const Eigen::MatrixXd& _X, const Eigen::MatrixXd& _Y);
    void optimize();

    Eigen::VectorXd predict(const Eigen::VectorXd& _x);
    Eigen::VectorXd varianceOfLastPrediction() const { return mVariance; }
    
protected:
    GaussianProcessImp* imp;
    Eigen::VectorXd mVariance;
}; // class GaussianProcess

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_GAUSSIANPROCESS_H

