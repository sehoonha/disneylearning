/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh

 * Implementation use the modified version of the following library
 * https://github.com/SheffieldML/GPc
 * - changes: update Makefile to create libGpc.a
 * Library removed and changed to libgp
 * https://github.com/mblum/libgp
 */

#include "GaussianProcess.h"
// #include "GPc/GPc.h" // Gaussian Process c library
#include "gp.h"
#include "gp_utils.h"
#include "cg.h"
#include "CGMulti.h"
#include "utils/Misc.h"
#include <cstdio>
#include <boost/thread.hpp>

#include "utils/CppCommon.h"
#include "utils/Option.h"

namespace disney {
namespace learning {

struct GaussianProcessImp {
    // One GP for one dimension
    std::vector<libgp::GaussianProcess*> gp_array;
    Eigen::MatrixXd X;
    Eigen::MatrixXd Y;

    GaussianProcessImp() {}
    ~GaussianProcessImp() {}

    int numData() { return X.rows(); }
    int numDimInput() { return X.cols(); }
    int numDimOutput() { return Y.cols(); }
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
    int N = _X.rows();
    int NINPUT = _X.cols();
    int NOUTPUT = _Y.cols();

    imp->X = _X;
    imp->Y = _Y;

    // cout << imp->X << endl;
    LOG(INFO) << "data is copied";
    const std::string COV = utils::Option::read("simulation.gp.cov").toString();
    LOG(INFO) << "Covariance Function = " << COV;

    for (int i = 0; i < NOUTPUT; i++) {
        libgp::GaussianProcess* gp = new libgp::GaussianProcess(NINPUT, COV.c_str());
        // libgp::GaussianProcess* gp = new libgp::GaussianProcess(NINPUT, "CovSum (CovSEard, CovNoise)");


        // libgp::GaussianProcess* gp = new libgp::GaussianProcess(NINPUT, "CovSum ( CovSEiso, CovNoise)");
        // libgp::GaussianProcess* gp = new libgp::GaussianProcess(NINPUT, "CovSum ( CovSEard, CovNoise)");
        // libgp::GaussianProcess* gp = new libgp::GaussianProcess(NINPUT, "CovSum ( CovLinearard, CovNoise)");
        // libgp::GaussianProcess* gp = new libgp::GaussianProcess(NINPUT, "CovSum ( CovMatern5iso, CovNoise)");
        // libgp::GaussianProcess* gp = new libgp::GaussianProcess(NINPUT, "CovSum ( CovRQiso, CovNoise)");



        Eigen::VectorXd params = Eigen::VectorXd::Zero(gp->covf().get_param_dim());
        params( params.size() - 1) = -2.0;

        utils::OptionItem opt = utils::Option::read("simulation.gp");
        if (opt.hasAttr("initParams")) {
            std::vector<double> vparams = opt.attrVectorDouble("initParams");
            Eigen::Map<Eigen::VectorXd> mparams( vparams.data(), vparams.size());
            CHECK_EQ( (int)mparams.size(), (int)gp->covf().get_param_dim() );
            params = mparams;
            LOG(INFO) << "Initial params = " << params.transpose();
        }

        gp->covf().set_loghyper(params);
        imp->gp_array.push_back(gp);
        LOG(INFO) << "GaussianProcess #" << i + 1 << " / " << NOUTPUT << " is initialized";
    }
    LOG(INFO) << "create all GPs structures";

    double* x = new double[NINPUT + 2];
    for (int i = 0; i < N; i++) {
        const Eigen::VectorXd& input = (imp->X).row(i);
        for (int j = 0; j < NINPUT; j++) {
            x[j] = input[j];
        }

        const Eigen::VectorXd& output = (imp->Y).row(i);
        for (int j = 0; j < NOUTPUT; j++) {
            double y = output(j);
            libgp::GaussianProcess* gp = imp->gp_array[j];
            gp->add_pattern(x, y);
        }
        LOG_EVERY_N(INFO, 1000) << i << "/" << N << " pattern is added";
    }
    delete[] x;

    
    LOG(INFO) << FUNCTION_NAME() << " OK: " << N << " "
              << imp->numDimInput() << " " << imp->numDimOutput();
}

void GaussianProcess::loadAll() {
    LOG(INFO) << FUNCTION_NAME() << " OK";
    int NOUTPUT = imp->numDimOutput();
    const int MAX_FILENAME = 256;
    for (int i = 0; i < NOUTPUT; i++) {
        libgp::GaussianProcess* gp = imp->gp_array[i];
        char filename[MAX_FILENAME];
        sprintf(filename, "gpmodel%d.gp", i);
        FILE* fp = fopen(filename, "r");
        if (fp != NULL) {
            delete gp;
            gp = new libgp::GaussianProcess(filename);
            imp->gp_array[i] = gp;
            LOG(INFO) << "loading GP " << i << " from " << filename;
        } else {
            LOG(INFO) << "cannot load GP " << i << " from " << filename;
        }
    }
    // test();
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void GaussianProcess::saveAll() {
    int NOUTPUT = imp->numDimOutput();
    const int MAX_FILENAME = 256;
    for (int i = 0; i < NOUTPUT; i++) {
        libgp::GaussianProcess* gp = imp->gp_array[i];
        char filename[MAX_FILENAME];
        sprintf(filename, "gpmodel%d.gp", i);
        LOG(INFO) << "writing GP " << i << " to " << filename;
        gp->write(filename);
    }
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void GaussianProcess::test() {
    int N = imp->numData();
    double sum_error = 0.0;
    for (int i = 0; i < N; i++) {
        Eigen::VectorXd x = imp->X.row(i);
        Eigen::VectorXd x2 = x + 0.01 * Eigen::VectorXd::Random(x.size());
        Eigen::VectorXd y = this->predict(x2);
        Eigen::VectorXd yhat = imp->Y.row(i);
        double v = this->varianceOfLastPrediction().norm();
        double err = (y - yhat).norm();
        sum_error += err;
        cout << endl;
        cout << i << " : "
             << err << ". " << endl
             << "x : " << x.transpose() << endl
             << "x2 : " << x2.transpose() << endl
             << "y : " << y.transpose() << endl
             << "f : " << yhat.transpose() << " "
             << endl;
        cout << "|var| = " << v << endl;
    }
    cout << sum_error << endl;
}

void GaussianProcess::setTrainingData(const Eigen::MatrixXd& _X, const Eigen::MatrixXd& _Y) {
}


void worker_optimizer(int index, libgp::GaussianProcess* gp, int MAX_OPT_LOOP, int VERBOSE) {
    LOG(INFO) << FUNCTION_NAME() << " ID = " << index;
    libgp::CG cg;
    cg.maximize(gp, MAX_OPT_LOOP, VERBOSE);
    LOG(INFO) << FUNCTION_NAME() << " ID = " << index << " OK";
}


void GaussianProcess::optimize() {
    LOG(INFO) << FUNCTION_NAME();

    int VERBOSE = 1;
    int MAX_OPT_LOOP = utils::Option::read("simulation.gp.maxOptLoop").toInt();
    LOG(INFO) << "MAX_OPT_LOOP = " << MAX_OPT_LOOP;

    // // Choice 1. Share the hyper parameters
    // libgp::CGMulti cg;
    // cg.maximize(imp->gp_array, MAX_OPT_LOOP, VERBOSE);

    // Choice 2. handle separately
    for (int i = 0; i < imp->gp_array.size(); i++) {
        libgp::GaussianProcess* gp = imp->gp_array[i];
        LOG(INFO) << "Params " << i << " = " << utils::V2S(gp->covf().get_loghyper());
    }

    VERBOSE = 0;
    std::vector<boost::thread*> threads;
    for (int i = 0; i < imp->gp_array.size(); i++) {
        libgp::GaussianProcess* gp = imp->gp_array[i];
        boost::thread* th = new boost::thread(&worker_optimizer, i, gp, MAX_OPT_LOOP, VERBOSE); 
        threads.push_back(th);
        // libgp::CG cg;
        // cg.maximize(gp, MAX_OPT_LOOP, VERBOSE);
    }
    for (int i = 0; i < threads.size(); i++) {
        boost::thread* th = threads[i];
        LOG(INFO) << "waiting for joining the thread " << i;
        th->join();
        LOG(INFO) << "success for joining the thread " << i;
    }

    for (int i = 0; i < imp->gp_array.size(); i++) {
        libgp::GaussianProcess* gp = imp->gp_array[i];
        LOG(INFO) << "Params " << i << " = " << utils::V2S(gp->covf().get_loghyper());
        
    }

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

Eigen::VectorXd GaussianProcess::predict(const Eigen::VectorXd& _x) {
    int NINPUT  = imp->numDimInput();
    int NOUTPUT = imp->numDimOutput();
    CHECK_EQ( (int)_x.size(), NINPUT );

    double* x = new double[NINPUT + 2];
    for (int j = 0; j < NINPUT; j++) {
        x[j] = _x(j);
    }

    Eigen::VectorXd ret(NOUTPUT);
    mVariance = Eigen::VectorXd::Zero(NOUTPUT);
    for (int j = 0; j < NOUTPUT; j++) {
        libgp::GaussianProcess* gp = imp->gp_array[j];
        ret(j) = gp->f(x);
        mVariance(j) = gp->var(x);
    }
    delete[] x;
    return ret;
}

Eigen::VectorXd GaussianProcess::hyperParameters() {
    for (int i = 0; i < imp->gp_array.size(); i++) {
        libgp::GaussianProcess* gp = imp->gp_array[i];
        return gp->covf().get_loghyper();
    }
    return Eigen::VectorXd::Zero(1);
}

void GaussianProcess::setHyperParameters(const Eigen::VectorXd& _hparams) {
    for (int i = 0; i < imp->gp_array.size(); i++) {
        libgp::GaussianProcess* gp = imp->gp_array[i];
        gp->covf().set_loghyper(_hparams);
    }
}

// class GaussianProcess ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disney


