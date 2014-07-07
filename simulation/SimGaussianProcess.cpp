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
#include "SimMathcalBongo.h"
#include "learning/GaussianProcess.h"


#define D_INPUT 6
#define D_OUTPUT 6
#define W_VEL 0.1
#define W_TOR 0.05

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
    double angIni = utils::Option::read("simulation.init.angle").toDouble();
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
    // std::string filename = "data_realsim.csv";
    std::string filename = utils::Option::read("simulation.gp.filename").toString();
    LOG(INFO) << "filename = " << filename;
    std::ifstream fin(filename.c_str());

    Eigen::VectorXd currState;
    Eigen::VectorXd currTorque;

    int dataRate = utils::Option::read("simulation.gp.dataRate").toInt();
    LOG(INFO) << "Data rate = " << dataRate;

    std::vector<Eigen::VectorXd> states;
    std::vector<Eigen::VectorXd> torques;
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
        states.push_back(currState);
        torques.push_back(currTorque);
    }
    fin.close();
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
        const double STEP_LENGTH_LIMIT = 1.0;
        if (stepLength < STEP_LENGTH_LIMIT && (loop % dataRate == 0) ) {

            Eigen::VectorXd input(D_INPUT);
            input(0) = currSimState(0);
            input(1) = currSimState(1);
            input(2) = currSimState(2);
            input(3) = W_VEL * currSimState(6 + 0);
            input(4) = W_VEL * currSimState(6 + 1);
            input(5) = W_VEL * currSimState(6 + 2);
            // input(0) = prevState(0);
            // input(1) = prevState(1);
            // input(2) = prevState(2);
            // input(3) = W_VEL * prevState(6 + 0);
            // input(4) = W_VEL * prevState(6 + 1);
            // input(5) = W_VEL * prevState(6 + 2);
            // input(6) = W_TOR * prevTorque(0);

            Eigen::VectorXd output = Eigen::VectorXd::Zero(D_OUTPUT);
            Eigen::VectorXd diff = currState;
            // Eigen::VectorXd diff = currState - currSimState;
            for (int i = 0; i < 3; i++) {
                output(i) = diff(i);
                output(i + 3) = W_VEL * diff(i + 6);
            }

            inputs.push_back(input);
            outputs.push_back(output);

            cout << "== " << loop << " ==" << endl;
            cout << stepLength << endl;
            cout << "input  = " << input.transpose() << endl;
            cout << "output = " << output.transpose() << endl;
            cout << currState.transpose() << endl << currSimState.transpose() << endl;
            cout << endl;
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
    Eigen::MatrixXd X(N, D_INPUT );
    Eigen::MatrixXd Y(N, D_OUTPUT );
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
    // gp->loadAll();

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
    // LOG(INFO) << FUNCTION_NAME();
    Eigen::VectorXd x_prev = mState;

    // 1. Generate delta from our siimulation model
    model->setState( mState );
    model->setTorque( mTorque );
    model->integrate();
    Eigen::VectorXd x_sim = model->state();
    Eigen::VectorXd dx = model->state() - x_prev;

    // 2. Correct the dynamics
    Eigen::VectorXd input( D_INPUT );
    input(0) = x_sim(0);
    input(1) = x_sim(1);
    input(2) = x_sim(2);
    input(3) = W_VEL * x_sim(6 + 0);
    input(4) = W_VEL * x_sim(6 + 1);
    input(5) = W_VEL * x_sim(6 + 2);
    // input(0) = x_prev(0);
    // input(1) = x_prev(1);
    // input(2) = x_prev(2);
    // input(3) = W_VEL * x_prev(6 + 0);
    // input(4) = W_VEL * x_prev(6 + 1);
    // input(5) = W_VEL * x_prev(6 + 2);
    // input(6) = W_TOR * mTorque(0);

    Eigen::VectorXd dx_delta = Eigen::VectorXd::Zero( 12 );
    if (gp) {
        // cout << endl;
        // cout << "input = " << input.transpose() << endl;
        Eigen::VectorXd output = gp->predict( input );
        Eigen::VectorXd var    = gp->varianceOfLastPrediction();
        if (var.norm() < 1.0) {
            for (int i = 0; i < 3; i++) {
                dx_delta(i) = output(i);
                dx_delta(i + 6) = (1.0 / W_VEL) * output(i + 3);
            }
            // LOG(INFO) << ">> "
            //           << "(" << var.norm() << " / " << var.transpose() << ") "
            //           << endl << dx_delta.transpose();
        } else {
            // LOG_EVERY_N(INFO, 30)
            //     << "reject due to high variance: "  
            //     << "(" << var.norm() << " / " << var.transpose() << ") ";
        }
        
        // cout << "model->state = " << model->state().transpose() << endl;
    }

    // // 3. To the next state
    // Eigen::VectorXd x_curr = mState + dx + dx_delta;
    // mState = x_curr;

    mState = dx_delta;
    

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


