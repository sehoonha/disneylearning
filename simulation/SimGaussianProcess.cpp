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
    LOG(INFO) << "W_VEL = " << W_VEL;
    LOG(INFO) << "W_TOR = " << W_TOR;

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

Eigen::VectorXd SimGaussianProcess::createInput(const Eigen::VectorXd& prevState,
                                                const Eigen::VectorXd& prevTorque,
                                                const Eigen::VectorXd& currSimState) {
    int ptr = 0;
    Eigen::VectorXd input(numDimInput());
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

    int dataRate = utils::Option::read("simulation.gp.dataRate").toInt();
    LOG(INFO) << "Data rate = " << dataRate;

    // Declare variables
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

        double stepLength = (currState - prevState).norm();
        double stateDifference = (currState - currSimState).norm();
        const double STEP_LENGTH_LIMIT = 1.0;
        const double STATE_DIFFERENCE_LIMIT = 10.0;
        if (stepLength < STEP_LENGTH_LIMIT
            && stateDifference < STATE_DIFFERENCE_LIMIT
            && fabs(prevState(0) + prevState(1) + prevState(2)) < 0.5
            && fabs(prevState(0)) + fabs(prevState(1)) + fabs(prevState(2)) < 4.5
            && (loop % dataRate == 0)
            ) {

            // 1. Create input
            Eigen::VectorXd input = createInput(prevState, prevTorque, currSimState);

            // 2. Create output
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
                    output(i) = diff(i);
                    output(i + 3) = W_VEL * diff(i + 6);
                }
            }

            // 3. Collect input/output
            inputs.push_back(input);
            outputs.push_back(output);

            // using disney::utils::V2S;
            // LOG(INFO) << "== " << loop << " ==";
            // LOG(INFO) << "input  = " << V2S(input);
            // LOG(INFO) << "output = " << V2S(output);
            // LOG(INFO) << "prevState = " << V2S(prevState);
            // LOG(INFO) << "currState = " << V2S(currState);
            // LOG(INFO) << "currSimState = " << V2S(currSimState);
        } else {
            // cout << "== " << loop << " ==" << endl;
            // cout << stepLength << endl;
            // cout << currState.transpose() << endl << currSimState.transpose() << endl;
            // cout << endl;
        }

        prevState  = currState;
        prevTorque = currTorque;
    }

    // Convert to the matrix form
    int N = inputs.size();
    Eigen::MatrixXd X(N, numDimInput() );
    Eigen::MatrixXd Y(N, numDimOutput() );
    for (int i = 0; i < N; i++) {
        X.row(i) = inputs[i];
        Y.row(i) = outputs[i];
    }

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
        Eigen::VectorXd output = gp->predict( input );
        Eigen::VectorXd var    = gp->varianceOfLastPrediction();

        for (int i = 0; i < 3; i++) {
            dx_delta(i) = output(i);
            dx_delta(i + 6) = (1.0 / W_VEL) * output(i + 3);
        }

        // Adjust the difference using the variance
        double v = var.norm();
        // double w = exp(-1000000.0 * v);
        // double w = exp(-100000.0 * v);
        double w = exp(-1000.0 * v);
        dx_delta *= w;

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


