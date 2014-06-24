/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "SimGaussianProcess.h"
#include <fstream>
#include "utils/CppCommon.h"
#include "SimMathcalBongo.h"
#include "learning/GaussianProcess.h"

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
    int n = numDimConfig();
    int m = numDimState();
    
    Eigen::VectorXd xEq(m);
    xEq << 0, 0, 0, 0, PI_2, -PI_2        , 0, 0, 0, 0, 0, 0;

    Eigen::VectorXd xOffset(m);
    double angIni = 0.3;
    xOffset << 0.0, (angIni * PI / 180), 0, 0, 0, 0        , 0, 0, 0, 0, 0, 0;
        
    mState = xEq + xOffset;

    mTorque = Eigen::VectorXd::Zero( numDimTorque() );

    setState(mState);
    clearHistory();


    model = (new SimMathcalBongo());
    model->init();

    LOG(INFO) << FUNCTION_NAME() << " OK";
    return this;
}


void SimGaussianProcess::train() {
    int n = numDimState();
    int m = numDimTorque();
    LOG(INFO) << FUNCTION_NAME() << " : " << n << ", " << m;
    std::string filename = "data_realsim.csv";
    std::ifstream fin(filename.c_str());

    Eigen::VectorXd prevState;
    Eigen::VectorXd prevTorque;
    Eigen::VectorXd currState;
    Eigen::VectorXd currTorque;

    std::vector<Eigen::VectorXd> inputs;
    std::vector<Eigen::VectorXd> outputs;
    for (int loop = 0; ; loop++) {
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

        // Now we have all prev/curr state/torques. ready for generating data.
        if (loop > 0) {
            // 1. Generate delta from our siimulation model
            model->setState( prevState );
            model->setTorque( prevTorque );
            model->integrate();
            Eigen::VectorXd currSimState = model->state();

            // Eigen::VectorXd input(n + m);
            // input.head(n) = prevState;
            // input.tail(m) = prevTorque;
            Eigen::VectorXd input(6);
            input(0) = prevState(0);
            input(1) = prevState(1);
            input(2) = prevState(2);
            input(3) = 0.1 * prevState(6 + 0);
            input(4) = 0.1 * prevState(6 + 1);
            input(5) = 0.1 * prevState(6 + 2);
            // input(6) = prevTorque(0);


            // Eigen::VectorXd output(n);
            // output = currState - currSimState;
            Eigen::VectorXd output = Eigen::VectorXd::Zero(6);
            Eigen::VectorXd diff = currState - currSimState;
            // diff = -diff;
            for (int i = 0; i < 3; i++) {
                output(i) = diff(i);
                output(i + 3) = 0.1 * diff(i + 6);
            }

            inputs.push_back(input);
            outputs.push_back(output);

            if (loop < 10000) {
                cout << "== " << loop << " ==" << endl;
                cout << input.transpose() << endl;
                cout << output.transpose() << endl;
                cout << currState.transpose() << endl << currSimState.transpose() << endl;
                cout << endl;
            }

        }

        prevState  = currState;
        prevTorque = currTorque;
    }
    fin.close();

    int N = inputs.size();
    Eigen::MatrixXd X(N, 6);
    Eigen::MatrixXd Y(N, 6);
    std::ofstream fout("training.csv");
    for (int i = 0; i < N; i++) {
        X.row(i) = inputs[i];
        Y.row(i) = outputs[i];
        for (int j = 0; j < X.cols(); j++) {
            fout << X(i, j) << " ";
        }
        for (int j = 0; j < Y.cols(); j++) {
            fout << Y(i, j) << " ";
        }
        fout << endl;
    }
    fout.close();
    LOG(INFO) << FUNCTION_NAME() << " : creating model... patience... ";
    gp = (new learning::GaussianProcess());
    gp->createModel(X, Y);
    gp->loadAll();
    // gp->optimize();

    // Finally, reset the used model...
    model->setState( mState );
    model->setTorque( mTorque );

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void SimGaussianProcess::optimize() {
    gp->optimize();
    gp->saveAll();
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void SimGaussianProcess::integrate() {
    Eigen::VectorXd x_prev = mState;

    // 1. Generate delta from our siimulation model
    model->setState( mState );
    model->setTorque( mTorque );
    model->integrate();
    Eigen::VectorXd dx = model->state() - x_prev;

    // 2. Correct the dynamics
    Eigen::VectorXd input( 6 );
    input(0) = x_prev(0);
    input(1) = x_prev(1);
    input(2) = x_prev(2);
    input(3) = 0.1 * x_prev(6 + 0);
    input(4) = 0.1 * x_prev(6 + 1);
    input(5) = 0.1 * x_prev(6 + 2);
    // input(3) = mTorque(0);

    Eigen::VectorXd dx_delta = Eigen::VectorXd::Zero( 12 );
    if (gp) {
        // cout << endl;
        // cout << "input = " << input.transpose() << endl;
        Eigen::VectorXd output = gp->predict( input );
        Eigen::VectorXd var    = gp->varianceOfLastPrediction();
        if (var.norm() < 10.0) {
            for (int i = 0; i < 3; i++) {
                if (var(i) < 0.0001) {
                    dx_delta(i) = output(i);
                }
                if (var(i + 3) < 0.0001) {
                    dx_delta(i + 6) = output(i + 3);
                }
            }
            LOG(INFO) << ">> "
                      << "(" << var.norm() << " / " << var.transpose() << ") "
                      << endl << dx_delta.transpose();
        } else {
            LOG_EVERY_N(INFO, 30)
                << "reject due to high variance: "  
                << "(" << var.norm() << " / " << var.transpose() << ") ";
        }
        
        // cout << "model->state = " << model->state().transpose() << endl;
    }

    // 3. To the next state
    Eigen::VectorXd x_curr = mState + dx + dx_delta;
    mState = x_curr;

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

