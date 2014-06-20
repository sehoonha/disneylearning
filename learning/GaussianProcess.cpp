/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 * Implementation use the modified version of the following library
 * https://github.com/SheffieldML/GPc
 * - changes: update Makefile to create libGpc.a
 */

#include "GaussianProcess.h"
#include "GPc/GPc.h" // Gaussian Process c library
#include "utils/CppCommon.h"

namespace disney {
namespace learning {

struct GaussianProcessImp {
    CGp* gp;
    CMatrix* X;
    CMatrix* Y;
    CCmpndKern* kern;
    CGaussianNoise* noise;

    GaussianProcessImp()
        : gp(NULL), X(NULL), Y(NULL), kern(NULL), noise(NULL)
        {}

    ~GaussianProcessImp() {
        if (noise) { delete noise; noise = NULL; }
        if (kern) { delete kern; kern = NULL; }
        if (Y) { delete Y; Y = NULL; }
        if (X) { delete X; X = NULL; }
        if (gp) { delete gp; gp = NULL; }
    };
    
    int numData() { if (X) return X->getRows(); else return 0; }
    int numDimInput() { if (gp) return gp->getInputDim(); else return 0; }
    int numDimOutput() { if (gp) return gp->getInputDim(); else return 0; }
};

////////////////////////////////////////////////////////////
// class GaussianProcess implementation
GaussianProcess::GaussianProcess()
    : imp(NULL)
{
    imp = new GaussianProcessImp;
}

GaussianProcess::~GaussianProcess() {
    if (imp) { delete imp; imp = NULL; }
}

void GaussianProcess::init() {
    // Hm....
}

void GaussianProcess::createModel(const Eigen::MatrixXd& _X, const Eigen::MatrixXd& _Y) {
    LOG(INFO) << FUNCTION_NAME();
    CHECK_NOTNULL(imp);
    CHECK_EQ(_X.rows(), _Y.rows());
    // Defines the dimensions
    int n = _X.rows();
    int dim_input = _X.cols();
    int dim_output = _Y.cols();

    // Copy data
    imp->X = new CMatrix(n, dim_input, 0.0);
    imp->Y = new CMatrix(n, dim_output, 0.0);

    CMatrix& X = *(imp->X);
    CMatrix& Y = *(imp->Y);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < dim_input; j++) {
            X.setVal( _X(i, j), i, j);
        }
        for (int j = 0; j < dim_output; j++) {
            Y.setVal( _Y(i, j), i, j);
        }
    }

    LOG(INFO) << "data is copied";

    // Create kernel and noise
    imp->kern = new CCmpndKern(X);
    CKern* defaultKern = new CRbfKern(X);
    CKern* biasKern = new CBiasKern(X);
    CKern* whiteKern = new CWhiteKern(X);
    imp->kern->addKern(defaultKern);
    imp->kern->addKern(biasKern);
    imp->kern->addKern(whiteKern);

    imp->noise = new CGaussianNoise(imp->Y);
    CMatrix noiseBias(1, Y.getCols(), 0.0);
    imp->noise->setBias(noiseBias);

    LOG(INFO) << "kernel and noise are initialized";

    imp->gp = new CGp(imp->kern, imp->noise, imp->X, CGp::FTC, 0, 3);
    imp->gp->setDefaultOptimiser(CGp::BFGS);
    LOG(INFO) << "gp class is initialized";

    LOG(INFO) << FUNCTION_NAME() << " OK: " << n << " " << dim_input << " " << dim_output;
}

void GaussianProcess::loadModel(const char* const filename) {
}

void GaussianProcess::saveModel(const char* const filename) {
}


void GaussianProcess::setTrainingData(const Eigen::MatrixXd& _X, const Eigen::MatrixXd& _Y) {
}

void GaussianProcess::optimize() {
    LOG(INFO) << FUNCTION_NAME();
    int iters = 1000;
    imp->gp->optimise(iters);
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

Eigen::VectorXd GaussianProcess::predict(const Eigen::VectorXd& _x) {
    int dim_input  = imp->numDimInput();
    int dim_output = imp->numDimOutput();

    CHECK_EQ( (int)_x.size(), dim_input );
    
    CMatrix PX(1, dim_input);
    for (int i = 0; i < dim_input; i++) {
        PX.setVal( _x(i), 0, i);
    }
    CMatrix PP(1, dim_output);
    CMatrix PY(1, dim_output);

    imp->gp->out(PY, PP, PX);

    Eigen::VectorXd ret(dim_output);
    for (int i = 0; i < dim_output; i++) {
        ret(i) = PY.getVal(0, i);
    }
    return ret;
}

// class GaussianProcess ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disney


