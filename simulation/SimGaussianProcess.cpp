/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "SimGaussianProcess.h"
#include <fstream>
#include "utils/CppCommon.h"
#include "utils/Option.h"
#include "utils/Misc.h"
#include "SimMathcalBongo.h"
#include "learning/GaussianProcess.h"


// #define D_INPUT 6
// #define D_OUTPUT 6
// #define W_VEL 0.1
// #define W_TOR 0.005

namespace disney {
namespace simulation {

////////////////////////////////////////////////////////////
// class SimGaussianProcess implementation
SimGaussianProcess::SimGaussianProcess()
    : Simulator(SIMTYPE_GAUSSIANPROCESS)
    , gp(NULL)
    , mIsInitedU(false)
{
}

SimGaussianProcess::~SimGaussianProcess() {
}

Simulator* SimGaussianProcess::init() {
    LOG(INFO) << FUNCTION_NAME();

    // Fetch flags
    utils::OptionItem o = utils::Option::read("simulation.gp.function");
    mFlagInputPrevState = o.attrBool("inputPrevState");
    mFlagInputCurrSimState = o.attrBool("inputCurrSimState");
    mFlagInputTorque = o.attrBool("inputTorque");
    mFlagOutputDiff = o.attrBool("outputDiff");

    LOG(INFO) << "mFlagInputPrevState = " << mFlagInputPrevState;
    LOG(INFO) << "mFlagInputCurrSimState = " << mFlagInputCurrSimState;
    LOG(INFO) << "mFlagInputTorque = " << mFlagInputTorque;
    LOG(INFO) << "mFlagOutputDiff = " << mFlagOutputDiff;

    // Fetch weights
    utils::OptionItem o_w = utils::Option::read("simulation.gp.weight");
    W_VEL = o_w.attrDouble("vel");
    W_TOR = o_w.attrDouble("tor");
    K_DECAY = o_w.attrDouble("decay");
    LOG(INFO) << "W_VEL = " << W_VEL;
    LOG(INFO) << "W_TOR = " << W_TOR;
    LOG(INFO) << "K_DECAY = " << K_DECAY;

    // Calculate the dimensions
    mDimInput = 0;
    if (mFlagInputPrevState) mDimInput += 6;
    if (mFlagInputCurrSimState) mDimInput += 6;
    if (mFlagInputTorque) mDimInput += 1;
    mDimOutput = 6;
    LOG(INFO) << "numDimInput = " << numDimInput();
    LOG(INFO) << "numDimOutput = " << numDimOutput();
    
    // Setup the initial state

    int n = numDimConfig();
    int m = numDimState();
    
    Eigen::VectorXd xEq(m);
    xEq << 0, 0, 0, 0, PI_2, -PI_2        , 0, 0, 0, 0, 0, 0;

    Eigen::VectorXd xOffset(m);
    Eigen::VectorXd stateIni = utils::Option::read("simulation.init.state").toVectorXd();
    if (utils::Option::read("simulation.init.unitIsDeg").toBool()) {
        stateIni *= (PI / 180.0);
    }
    xOffset = stateIni;
    // double angIni = utils::Option::read("simulation.init.angle").toDouble();
    // xOffset << 0.0, (angIni * PI / 180), 0, 0, 0, 0        , 0, 0, 0, 0, 0, 0;
        
    mState = xEq + xOffset;

    mTorque = Eigen::VectorXd::Zero( numDimTorque() );

    setState(mState);
    clearHistory();


    model = (new SimMathcalBongo());
    model->init();
    model->setType(SIMTYPE_GAUSSIANPROCESS"_INT"); // Set type as Internal

    LOG(INFO) << FUNCTION_NAME() << " OK";
    return this;
}

bool SimGaussianProcess::isGoodInput(int index, 
                                     const Eigen::VectorXd& prevState,
                                     const Eigen::VectorXd& currState,
                                     const Eigen::VectorXd& currSimState,
                                     bool includeTesting ) {
    double stepLength = (currState - prevState).norm();
    const double STEP_LENGTH_MIN = 0.000001;
    const double STEP_LENGTH_MAX = 1.0;
    if (stepLength < STEP_LENGTH_MIN) return false;
    if (stepLength > STEP_LENGTH_MAX) return false;

    double stateDifference = (currState - currSimState).norm();
    const double STATE_DIFFERENCE_MAX = 1.0;
    if (stateDifference > STATE_DIFFERENCE_MAX) return false;

    double tilt = fabs(prevState(0) + prevState(1) + prevState(2));
    const double TILT_MAX = 0.7;
    if (tilt > TILT_MAX) return false;

    double distToEquilibrium = fabs(prevState(0)) + fabs(prevState(1)) + fabs(prevState(2));
    const double DIST_TO_EQUILIBRIUM_MAX = 4.5;
    if (distToEquilibrium > DIST_TO_EQUILIBRIUM_MAX) return false;


    bool isCorrectFrame = ((index % dataRate) == 0);
    if (// includeTesting &&
        ((index % dataRate) == (dataRate / 2))) {
        isCorrectFrame = true;
    }
    if (!isCorrectFrame) return false;

    return true;


    // if (stepLength < STEP_LENGTH_LIMIT
    //     && stateDifference < STATE_DIFFERENCE_LIMIT
    //     && fabs(prevState(0) + prevState(1) + prevState(2)) < 0.5
    //     && fabs(prevState(0)) + fabs(prevState(1)) + fabs(prevState(2)) < 4.5
    //     && ((prevState -  currState).norm() > 0.000001)
    //     // && (prevTorque.norm() < 1.0)
    //     // && (prevTorque(0) > 15)
    //     && ( ((index % dataRate) == 0) || ((index % dataRate) == (dataRate / 2)) ) ) {
    //     return true;
    // }
    // return false;

}

Eigen::VectorXd SimGaussianProcess::createInput(const Eigen::VectorXd& prevState,
                                                const Eigen::VectorXd& prevTorque,
                                                const Eigen::VectorXd& currSimState) {
    int ptr = 0;
    Eigen::VectorXd input(numDimInput());
    // input(ptr++) = prevState(0);
    // input(ptr++) = prevState(1);
    // input(ptr++) = prevState(2);
    // input(ptr++) = W_VEL * currSimState(6 + 0);
    // input(ptr++) = W_VEL * currSimState(6 + 1);
    // input(ptr++) = W_VEL * currSimState(6 + 2);
    // input(ptr++) = W_TOR * prevTorque(0);

    if (mFlagInputPrevState) {
        input(ptr++) = prevState(0);
        input(ptr++) = prevState(1);
        input(ptr++) = prevState(2);
        input(ptr++) = W_VEL * prevState(6 + 0);
        input(ptr++) = W_VEL * prevState(6 + 1);
        input(ptr++) = W_VEL * prevState(6 + 2);
    }
    if (mFlagInputCurrSimState) {
        input(ptr++) = currSimState(0);
        input(ptr++) = currSimState(1);
        input(ptr++) = currSimState(2);
        input(ptr++) = W_VEL * currSimState(6 + 0);
        input(ptr++) = W_VEL * currSimState(6 + 1);
        input(ptr++) = W_VEL * currSimState(6 + 2);
    }
    if (mFlagInputTorque) {
        input(ptr++) = W_TOR * prevTorque(0);
    }
    CHECK_EQ( (int)ptr, (int)numDimInput() );
    return input;
}

Eigen::VectorXd SimGaussianProcess::createOutput(const Eigen::VectorXd& currState,
                                                 const Eigen::VectorXd& currSimState) {
    Eigen::VectorXd output = Eigen::VectorXd::Zero(numDimOutput());
    if (mFlagOutputDiff) {
        Eigen::VectorXd diff = currState - currSimState;
        for (int i = 0; i < 3; i++) {
            output(i) = diff(i);
            output(i + 3) = W_VEL * diff(i + 6);
        }
    } else {
        Eigen::VectorXd diff = currState;
        for (int i = 0; i < 3; i++) {
            // output(i) = W_VEL * diff(i + 6);
            output(i) = diff(i);
            output(i + 3) = W_VEL * diff(i + 6);
        }
    }
    return output;
}

void SimGaussianProcess::train() {
    int n = numDimState();
    int m = numDimTorque();
    LOG(INFO) << FUNCTION_NAME() << " : " << n << ", " << m;


    // // std::string filename = "data_realsim.csv";
    // std::string filename = utils::Option::read("simulation.gp.filename").toString();
    // LOG(INFO) << "filename = " << filename;
    // std::ifstream fin(filename.c_str());

    Eigen::VectorXd currState;
    Eigen::VectorXd currTorque;

    int dataRate = utils::Option::read("simulation.gp.dataRate").toInt();
    LOG(INFO) << "Data rate = " << dataRate;

    std::vector<Eigen::VectorXd> states;
    std::vector<Eigen::VectorXd> torques;

    FOREACH(const utils::OptionItem& o, utils::Option::readAll("simulation.gp.data")) {
        const std::string filename = o.attrString("filename");
        LOG(INFO) << "Data filename = [" << filename << "]";
        std::ifstream fin(filename.c_str());

        int loop = 0;
        for (; ; loop++) {
            currState  = Eigen::VectorXd::Zero(n);
            currTorque = Eigen::VectorXd::Zero(m);

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
            states.push_back(currState);
            torques.push_back(currTorque);
        }
        fin.close();

        LOG(INFO) << "Data filename = [" << filename << "] : # lines = " << loop;
        LOG(INFO) << "--> # states = " << states.size();
    }
    LOG(INFO) << FUNCTION_NAME() << " OK";
    train(states, torques);
}

void SimGaussianProcess::train(const std::vector<Eigen::VectorXd>& states,
                               const std::vector<Eigen::VectorXd>& torques) {
    // Define and checking dimensions
    int n = numDimState();
    int m = numDimTorque();
    int NDATA = states.size();
    LOG(INFO) << FUNCTION_NAME();
    LOG(INFO) << "dimensions : " << n << ", " << m;
    LOG(INFO) << "num data : " << NDATA;

    CHECK_LT( 0, NDATA );
    CHECK_EQ( (int)states.size(), (int)torques.size() );
    CHECK_EQ( n, (int)states[0].size() );
    CHECK_EQ( m, (int)torques[0].size() );
    LOG(INFO) << FUNCTION_NAME() << " : data size checking okay";

    dataRate = utils::Option::read("simulation.gp.dataRate").toInt();
    LOG(INFO) << "Data rate = " << dataRate;

    // Declare variables
    Eigen::VectorXd prevState;
    Eigen::VectorXd prevTorque;
    Eigen::VectorXd currState;
    Eigen::VectorXd currTorque;


    std::vector<Eigen::VectorXd> inputs;
    std::vector<Eigen::VectorXd> outputs;

    std::vector<Eigen::VectorXd> testinputs;
    std::vector<Eigen::VectorXd> testoutputs;

    for (int loop = 0; loop < NDATA; loop++) {
        currState  = states[loop];
        currTorque = torques[loop];

        if (loop == 0) {
            prevState  = currState;
            prevTorque = currTorque;
            continue;
        }

        // Now we have all prev/curr state/torques. ready for generating data.
        // 1. Generate delta from our siimulation model
        model->setState( prevState );
        model->setTorque( prevTorque );
        model->integrate();
        Eigen::VectorXd currSimState = model->state();

        if (isGoodInput(loop, prevState, currState, currSimState, true)) {
            // 1. Create input
            Eigen::VectorXd input = createInput(prevState, prevTorque, currSimState);

            // 2. Create output
            Eigen::VectorXd output = createOutput(currState, currSimState);
            // 3. Collect input/output
            if ( (loop % dataRate) == 0) {
                inputs.push_back(input);
                outputs.push_back(output);
                LOG(INFO) << "input/output" << endl
                          << utils::V2S(input) << endl
                          << utils::V2S(output);
            } else {
                testinputs.push_back(input);
                testoutputs.push_back(output);
            }
        } 

        prevState  = currState;
        prevTorque = currTorque;
    }


    // Convert to the matrix form
    int N = inputs.size();
    LOG(INFO) << "# of inputs/outputs = " << N;
    Eigen::MatrixXd X(N, numDimInput() );
    Eigen::MatrixXd Y(N, numDimOutput() );
    for (int i = 0; i < N; i++) {
        X.row(i) = inputs[i];
        Y.row(i) = outputs[i];
    }

    int M = testinputs.size();
    LOG(INFO) << "# of test inputs/outputs = " << M;
    Eigen::MatrixXd P(M, numDimInput() );
    Eigen::MatrixXd Q(M, numDimOutput() );
    for (int i = 0; i < M; i++) {
        P.row(i) = testinputs[i];
        Q.row(i) = testoutputs[i];
    }

    bool SVDEnabled = utils::Option::read("simulation.gp.svd.enabled").toBool();
    LOG(INFO) << "SVD enabled = " << SVDEnabled;
    if (SVDEnabled) {
        int selectedAxes = utils::Option::read("simulation.gp.svd.selectedAxes").toInt();
        LOG(INFO) << "SVD: # of selected axes = " << selectedAxes;
        Eigen::MatrixXd Xr = X.transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Xr, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd singularValues = svd.singularValues();
        U = svd.matrixU();
        U = U.leftCols(selectedAxes);
        LOG(INFO) << ">>> perform SVD";
        LOG(INFO) << "Singular values: " << utils::V2S(singularValues, 6);
        LOG(INFO) << "size of U = " << U.rows() << " " << U.cols();

        Xr = (U.transpose() * Xr);
        Xr.transposeInPlace();
        X = Xr;
        mIsInitedU = true;
        // LOG(INFO) << endl << X;
        // exit(0);
    }
    
    // {
    //     Eigen::MatrixXd Xr = X.transpose();
    //     Eigen::JacobiSVD<Eigen::MatrixXd> svd(Xr, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //     cout << "Its singular values are:" << endl << svd.singularValues() << endl;
    //     cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
    //     cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;        
    //     exit(0);
    // }



    // Clear the structure
    if (gp) {
        delete gp;
        gp = NULL;
        LOG(INFO) << FUNCTION_NAME() << " : delete the previous Gaussian Process";
    }

    // Create the GP structure
    LOG(INFO) << FUNCTION_NAME() << " : creating model... patience... ";
    gp = (new learning::GaussianProcess());
    gp->createModel(X, Y);
    LOG(INFO) << FUNCTION_NAME() << " : creating model... NUM DATA = " << N;

    // {
    //     gp->optimize();
    //     Eigen::MatrixXd Pr = P.transpose();
    //     Pr = (U.transpose() * Pr).topRows(3);
    //     Pr.transposeInPlace();
        
    //     Eigen::MatrixXd S(M, numDimInput() );
    //     for (int i = 0; i < M; i++) {
    //         Eigen::VectorXd x = P.row(i);
    //         x += 0.3 * Eigen::VectorXd::Random(numDimInput());
    //         S.row(i) = x;
    //     }
        
    //     Eigen::MatrixXd Sr = S.transpose();
    //     Sr = (U.transpose() * Sr).topRows(3);
    //     Sr.transposeInPlace();

    //     Eigen::MatrixXd R(M, numDimOutput() );
    //     for (int i = 0; i < M; i++) {
    //         Eigen::VectorXd x = Sr.row(i);
    //         Eigen::VectorXd y = gp->predict(x);
    //         Eigen::VectorXd var = gp->varianceOfLastPrediction();
    //         Eigen::VectorXd ybar = Q.row(i);
    //         // if (var.norm() > 0.0001) {
    //         //     y.setZero();
    //         // }
    //         R.row(i) = y;

    //         using disney::utils::V2S;
    //         LOG(INFO) << "Case " << i;
    //         LOG(INFO) << "predict: " << V2S(y);
    //         LOG(INFO) << "answer : " << V2S(ybar);
    //         LOG(INFO) << "variance : " << "|" << var.norm() << "| <- " << V2S(var);
    //     }

        
    //     std::stringstream sout;
    //     sout << "hold on; " << endl;
    //     sout << "quiver3(" << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Pr(i, 0) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Pr(i, 1) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Pr(i, 2) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 0) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 1) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 2) << ","; sout << "]" << endl;
    //     sout << ", scale=1.0, 'color', [0, 0, 1]);" << endl;
    //     sout << "quiver3(" << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Sr(i, 0) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Sr(i, 1) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Sr(i, 2) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << R(i, 0) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << R(i, 1) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << R(i, 2) << ","; sout << "]" << endl;
    //     sout << ", scale=1.0, 'color', [1, 0, 0]);" << endl;
    //     sout << "xlabel(\"wheel\", \"fontsize\", 20)" << endl;
    //     sout << "ylabel(\"board\", \"fontsize\", 20)" << endl;
    //     sout << "zlabel(\"joint\", \"fontsize\", 20)" << endl;
    //     sout << "hold off; " << endl;

    //     LOG(INFO) << endl << endl << sout.str();
    //     std::ofstream fout("field.m");
    //     fout << endl << endl << sout.str();
    //     fout.close();
    //     exit(0);
    // }


    // {
    //     gp->optimize();
    //     Eigen::MatrixXd S(M, numDimInput() );
    //     Eigen::MatrixXd R(M, numDimOutput() );
    //     for (int i = 0; i < M; i++) {
    //         Eigen::VectorXd x = P.row(i);
    //         x += 0.3 * Eigen::VectorXd::Random(numDimInput());
    //         S.row(i) = x;
    //         Eigen::VectorXd y = gp->predict(x);
    //         Eigen::VectorXd var = gp->varianceOfLastPrediction();
    //         Eigen::VectorXd ybar = Q.row(i);
    //         // if (var.norm() > 0.0001) {
    //         //     y.setZero();
    //         // }
    //         R.row(i) = y;

    //         using disney::utils::V2S;
    //         LOG(INFO) << "Case " << i;
    //         LOG(INFO) << "predict: " << V2S(y);
    //         LOG(INFO) << "answer : " << V2S(ybar);
    //         LOG(INFO) << "variance : " << "|" << var.norm() << "| <- " << V2S(var);
    //     }


    //     std::stringstream sout;
    //     sout << "hold on; " << endl;
    //     // sout << "quiver3(" << endl;
    //     // sout << "["; for (int i = 0; i < N; i++) sout << X(i, 0) << ","; sout << "]," << endl;
    //     // sout << "["; for (int i = 0; i < N; i++) sout << X(i, 1) << ","; sout << "]," << endl;
    //     // sout << "["; for (int i = 0; i < N; i++) sout << X(i, 2) << ","; sout << "]," << endl;
    //     // sout << "["; for (int i = 0; i < N; i++) sout << Y(i, 0) << ","; sout << "]," << endl;
    //     // sout << "["; for (int i = 0; i < N; i++) sout << Y(i, 1) << ","; sout << "]," << endl;
    //     // sout << "["; for (int i = 0; i < N; i++) sout << Y(i, 2) << ","; sout << "]" << endl;
    //     // sout << ", scale=2000.0";
    //     // sout << ");" << endl;
    //     sout << "quiver3(" << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << P(i, 0) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << P(i, 1) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << P(i, 2) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 0) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 1) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 2) << ","; sout << "]" << endl;
    //     sout << ", scale=1.0, 'color', [0, 0, 1]);" << endl;

    //     sout << "quiver3(" << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << S(i, 0) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << S(i, 1) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << S(i, 2) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << R(i, 0) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << R(i, 1) << ","; sout << "]," << endl;
    //     sout << "["; for (int i = 0; i < M; i++) sout << R(i, 2) << ","; sout << "]" << endl;
    //     sout << ", scale=1.0, 'color', [1, 0, 0]);" << endl;

    //     sout << "xlabel(\"wheel\", \"fontsize\", 20)" << endl;
    //     sout << "ylabel(\"board\", \"fontsize\", 20)" << endl;
    //     sout << "zlabel(\"joint\", \"fontsize\", 20)" << endl;
    //     sout << "hold off; " << endl;

    //     LOG(INFO) << endl << endl << sout.str();
    //     std::ofstream fout("field.m");
    //     fout << endl << endl << sout.str();
    //     fout.close();
    //     exit(0);
    // }


    // Load the GP, if exists
    bool loadAtConsume = utils::Option::read("simulation.gp.behavior").attrBool("loadAtConsume");
    LOG(INFO) << "loadAtConsume = " << loadAtConsume;
    if (loadAtConsume) {
        gp->loadAll();
    }

    // Finally, reset the used model...
    model->setState( mState );
    model->setTorque( mTorque );
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void SimGaussianProcess::optimize() {
    gp->optimize();
    bool saveAfterOptimize = utils::Option::read("simulation.gp.behavior").attrBool("saveAfterOptimize");
    LOG(INFO) << "saveAfterOptimize = " << saveAfterOptimize;
    if (saveAfterOptimize) {
        gp->saveAll();
    }
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void SimGaussianProcess::testVectorField3D() {
    int n = numDimState();
    int m = numDimTorque();

    std::vector<Eigen::VectorXd> states;
    std::vector<Eigen::VectorXd> torques;

    // 1. read all states
    FOREACH(const utils::OptionItem& o, utils::Option::readAll("simulation.gp.test")) {
        const std::string filename = o.attrString("filename");
        LOG(INFO) << "Test filename = [" << filename << "]";
        std::ifstream fin(filename.c_str());

        int loop = 0;
        for (; ; loop++) {
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
            states.push_back(currState);
            torques.push_back(currTorque);
        }
        fin.close();

        LOG(INFO) << "Test filename = [" << filename << "] : # lines = " << loop;
        LOG(INFO) << "--> # states = " << states.size();
    }

    // 2. create input/output
    int NDATA = states.size();
    Eigen::VectorXd prevState;
    Eigen::VectorXd prevTorque;
    Eigen::VectorXd currState;
    Eigen::VectorXd currTorque;

    std::vector<Eigen::VectorXd> inputs;
    std::vector<Eigen::VectorXd> outputs;

    for (int loop = 0; loop < NDATA; loop++) {
        currState  = states[loop];
        currTorque = torques[loop];

        if (loop == 0) {
            prevState  = currState;
            prevTorque = currTorque;
            continue;
        }

        // Now we have all prev/curr state/torques. ready for generating data.
        // 1. Generate delta from our siimulation model
        model->setState( prevState );
        model->setTorque( prevTorque );
        model->integrate();
        Eigen::VectorXd currSimState = model->state();

        if (isGoodInput(loop + (dataRate / 2)
                        , prevState, currState, currSimState, false)) {
            // 1. Create input
            Eigen::VectorXd input = createInput(prevState, prevTorque, currSimState);

            // 2. Create output
            Eigen::VectorXd output = createOutput(currState, currSimState);

            // 3. Collect
            inputs.push_back(input);
            outputs.push_back(output);
        }

        prevState  = currState;
        prevTorque = currTorque;
    }

    // 3. Convert to the matrix form
    int M = inputs.size();
    Eigen::MatrixXd P(M, numDimInput() );
    Eigen::MatrixXd Q(M, numDimOutput() );
    for (int i = 0; i < M; i++) {
        P.row(i) = inputs[i];
        Q.row(i) = outputs[i];
    }


    // 4. Testing
    Eigen::MatrixXd S(M, numDimInput() );
    Eigen::MatrixXd R(M, numDimOutput() );
    for (int i = 0; i < M; i++) {
        Eigen::VectorXd x = P.row(i);
        x += 0.0 * Eigen::VectorXd::Random(numDimInput());
        if (mIsInitedU) {
            x = (U.transpose() * x).transpose();
        }
        S.row(i) = x;
        Eigen::VectorXd y = gp->predict(x);
        Eigen::VectorXd var = gp->varianceOfLastPrediction();
        Eigen::VectorXd ybar = Q.row(i);
        double v = var.norm();
        double w = exp(-K_DECAY * v);
        y *= w;

        // if (var.norm() > 0.0001) {
        //     y.setZero();
        // }
        R.row(i) = y;

        using disney::utils::V2S;
        LOG(INFO) << "Case " << i;
        LOG(INFO) << "input " << V2S(x);
        LOG(INFO) << "predict: " << V2S(y);
        LOG(INFO) << "answer : " << V2S(ybar);
        LOG(INFO) << "variance : " << "|" << var.norm() << ", " << w << "| <- " << V2S(var);
    }

    if (mIsInitedU) {
        P = (U.transpose() * P.transpose()).transpose();
    }
    
    std::stringstream sout;
    sout << "hold on; " << endl;
    sout << "quiver3(..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << P(i, 0) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << P(i, 1) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << P(i, 2) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 3) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 4) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << Q(i, 5) << ","; sout << "],..." << endl;
    sout << "'color', [0, 0, 1]);" << endl;
    sout << "quiver3(..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << S(i, 0) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << S(i, 1) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << S(i, 2) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << R(i, 3) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << R(i, 4) << ","; sout << "],..." << endl;
    sout << "["; for (int i = 0; i < M; i++) sout << R(i, 5) << ","; sout << "],..." << endl;
    sout << "'color', [1, 0, 0]);" << endl;
    sout << "grid on;" << endl;
    sout << "xlabel('wheel', 'fontsize', 20)" << endl;
    sout << "ylabel('board', 'fontsize', 20)" << endl;
    sout << "zlabel('joint', 'fontsize', 20)" << endl;
    sout << "hold off; " << endl;

    LOG(INFO) << endl << endl << sout.str();
    std::ofstream fout("field.m");
    fout << endl << endl << sout.str();
    fout.close();
        

    LOG(INFO) << FUNCTION_NAME() << " OK";
    exit(0);
}

void SimGaussianProcess::integrate() {
    // LOG(INFO) << FUNCTION_NAME();
    Eigen::VectorXd x_prev = mState;

    // 1. Generate delta from our siimulation model
    model->setState( mState );
    model->setTorque( mTorque );
    model->integrate();
    Eigen::VectorXd x_sim = model->state();
    Eigen::VectorXd dx = model->state() - x_prev;

    // 2. Correct the dynamics
    Eigen::VectorXd input = createInput(mState, mTorque, x_sim);

    Eigen::VectorXd dx_delta = Eigen::VectorXd::Zero( 12 );
    if (gp) {
        // cout << endl;
        // cout << "input = " << input.transpose() << endl;
        if (mIsInitedU) {
            input = (U.transpose() * input).transpose();
        }

        Eigen::VectorXd output = gp->predict( input );
        Eigen::VectorXd var    = gp->varianceOfLastPrediction();

        
        double h = this->timeStep();
        for (int i = 0; i < 3; i++) {
            // dx_delta(i + 6) = (1.0 / W_VEL) * output(i);
            // dx_delta(i) = h * dx_delta(i + 6);

            dx_delta(i) = output(i);
            dx_delta(i + 6) = (1.0 / W_VEL) * output(i + 3);
        }

        // LOG(INFO) << "before = " << utils::V2S(dx_delta, 6);
        // Adjust the difference using the variance
        double v = var.norm();
        // double w = exp(-1000000.0 * v);
        // double w = exp(-100000.0 * v);
        // double w = exp(-1000.0 * v);
        // double w = exp(-50.0 * v);
        double w = exp(-K_DECAY * v);
        // double w = 1.0;
        // dx_delta = w * dx_delta + (1 - w) * x_sim;
        dx_delta = w * dx_delta;

        // LOG(INFO) << endl;
        // LOG(INFO) << "Input: " << utils::V2S(input);
        // LOG(INFO) << "Output: " << utils::V2S(output);
        // LOG(INFO) << "dx_delta = " << utils::V2S(dx_delta, 6);
        // LOG(INFO) << "|var|, w = " << v << ", " << w;

        // LOG_EVERY_N(INFO, 20) << "|var|, w = " << v << ", " << w;

        // if (var.norm() < 0.0001) {
        //     for (int i = 0; i < 3; i++) {
        //         dx_delta(i) = output(i);
        //         dx_delta(i + 6) = (1.0 / W_VEL) * output(i + 3);
        //     }
        //     // LOG(INFO) << ">> "
        //     //           << "(" << var.norm() << " / " << var.transpose() << ") "
        //     //           << endl << dx_delta.transpose();
        // } else {
        //     // LOG_EVERY_N(INFO, 30)
        //     //     << "reject due to high variance: "  
        //     //     << "(" << var.norm() << " / " << var.transpose() << ") ";
        // }
        
        // cout << "model->state = " << model->state().transpose() << endl;
    }

    // 3. To the next state
    if (mFlagOutputDiff) {
        Eigen::VectorXd x_curr = mState + dx + dx_delta;
        mState = x_curr;
    } else {
        mState = dx_delta;
        // mState(6) = dx_delta(6);
        // mState(7) = dx_delta(7);
        // mState(8) = dx_delta(8);

        // mState(0) += h * dx_delta(6);
        // mState(1) += h * dx_delta(7);
        // mState(2) += h * dx_delta(8);

    }
    

    // 4. Finalize the state by maintaining the constraints
    // // Hard coded constraint..
    mState(3) = mState(2);
    mState(4) =  0.5 * PI - mState(2);
    mState(5) = -0.5 * PI - mState(2);

    mState(9) = mState(8);
    mState(10) = -mState(8);
    mState(11) = -mState(8);

    // cout << "final state = " << mState.transpose() << endl;

    model->setState( mState );
    // LOG(INFO) << FUNCTION_NAME() << " OK";
}

Eigen::VectorXd
SimGaussianProcess::deriv(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    return model->deriv(state, control);
}

void SimGaussianProcess::updateToHistory(int index) {
    Simulator::updateToHistory(index);
    model->setState( mState );
    model->setTorque( mTorque );
}

void SimGaussianProcess::render() {
    model->render();
}


// class SimGaussianProcess ends
////////////////////////////////////////////////////////////



} // namespace simulation
} // namespace disney


