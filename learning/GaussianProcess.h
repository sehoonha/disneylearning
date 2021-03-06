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
    void loadAll();
    void saveAll();
    void test();

    void setTrainingData(const Eigen::MatrixXd& _X, const Eigen::MatrixXd& _Y);
    void optimize();

    Eigen::VectorXd predict(const Eigen::VectorXd& _x);
    Eigen::VectorXd varianceOfLastPrediction() const { return mVariance; }

    Eigen::VectorXd hyperParameters();
    void setHyperParameters(const Eigen::VectorXd& _hparams);
    
    int numData() const;
protected:
    GaussianProcessImp* imp;
    Eigen::VectorXd mVariance;
}; // class GaussianProcess

} // namespace learning
} // namespace disney

#endif // #ifndef LEARNING_GAUSSIANPROCESS_H

