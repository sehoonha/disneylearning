/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Scripts.h"

#include <fstream>

#include "utils/CppCommon.h"
#include "utils/Misc.h"
#include "simulation/Manager.h"
#include "simulation/Simulator.h"
#include "simulation/SimMathcalBongo.h"
#include "simulation/SimBox2D.h"
#include "simulation/SimGaussianProcess.h"
#include "simulation/Evaluator.h"
#include "learning/GaussianProcess.h"

#include "Application.h"

namespace disney {
namespace app {

double interpolatedValue(int i, int N, double lo, double hi) {
    return lo + ((double)i / (double)(N - 1)) * (hi - lo);
}

void save(const Eigen::VectorXd& v, const char* const filename) {
    std::ofstream fout(filename);
    for (int i = 0; i < v.size(); i++) fout << v(i) << " ";
    fout << endl;
    fout.close();
}

void save(const Eigen::MatrixXd& m, const char* const filename) {
    std::ofstream fout(filename);
    for (int i = 0; i < m.rows(); i++) {
        for (int j = 0; j < m.cols(); j++) {
            fout << m(i, j) << " ";
        }
        fout << endl;
    }
    fout.close();
}

void plotVectorField(Application* app) {
    LOG(INFO) << FUNCTION_NAME();

    disney::simulation::Manager* manager = app->manager();
    int NS = manager->numSimulators();
    LOG(INFO) << "# of simulators = " << NS;
    CHECK_LT(0, NS);

    Eigen::VectorXd state = manager->simulator(0)->state();
    Eigen::VectorXd torque = Eigen::VectorXd::Zero(4);
    // torque << 20, 20, -20, -20;
    // torque << -20, -20, 20, 20;
    torque << 0, 0, -0, -0;
    LOG(INFO) << "state = " << utils::V2S(state, 4);
    LOG(INFO) << "torque = " << utils::V2S(torque, 4);
    std::vector<Eigen::VectorXd> stableStates;

    Eigen::MatrixXd DX0;
    Eigen::MatrixXd DY0;


    for (int loop = 0; loop < NS; loop++) {
        simulation::Simulator* s = manager->simulator(loop);

        int iN = 31;
        int jN = iN;
        double MINX = -0.05;
        double MAXX = 0.05;
        double MINY = -0.05;
        double MAXY = 0.05;
        double MARGIN = 0.0;

        Eigen::VectorXd X(iN);
        Eigen::VectorXd Y(jN);
        Eigen::MatrixXd DX(jN, iN);
        Eigen::MatrixXd DY(jN, iN);
        Eigen::MatrixXd V(jN, iN);
        
        for (int i = 0; i < iN; i++) {
            int ii = 1;
            double xi = interpolatedValue(i, iN, MINX, MAXX);
            state(ii) = X(i) = xi;
            state(0) = -xi;

            for (int j = 0; j < jN; j++) {
                int ji = 2;
                double xj = interpolatedValue(j, jN, MINY, MAXY);
                state(ji) = Y(j) = xj;
                state(3) = xj;
                state(4) = 0.5 * PI - xj;
                state(5) = -0.5 * PI - xj;

                Eigen::VectorXd stableState;
                Eigen::VectorXd nextState;
                // s->setState(state);
                // s->setTorque(torque);
                // s->integrate();
                // nextState = s->state();

                s->setState(state);
                for (int k = 0; k < 5; k++) {
                    s->setTorque(torque);
                    s->integrate();
                }
                nextState = s->state();
                

                
                // if (loop == 0) {
                //     s->setState(state);
                //     // for (int k = 0; k < 3; k++) {
                //     //     s->setTorque(Eigen::VectorXd::Zero(4));
                //     //     s->integrate();
                //     // }
                //     stableState = s->state();
                //     // stableState.tail(6).setZero();
                //     int idx = i * jN + j;
                //     CHECK_EQ( (int)idx, (int)stableStates.size() );
                //     stableStates.push_back(stableState);

                //     s->setTorque(torque);
                //     s->integrate();
                //     nextState = s->state();

                //     // LOG(INFO) << endl;
                //     // LOG(INFO) << utils::V2S(state, 4);
                //     // LOG(INFO) << utils::V2S(stableState, 4);
                //     // LOG(INFO) << utils::V2S(nextState, 4);
                // } else {
                //     int idx = i * jN + j;
                //     stableState = stableStates[idx];
                //     // LOG_EVERY_N(INFO, 100) << utils::V2S(stableState, 4);
                //     s->setState(stableState);
                //     s->setTorque(torque);
                //     s->integrate();
                //     nextState = s->state();
                    
                // }

                
                // Eigen::VectorXd nextState = s->state();
                // double dx_i = nextState(ii) - stableState(ii);
                // double dx_j = nextState(ji) - stableState(ji);
                double dx_i = nextState(ii) - state(ii);
                double dx_j = nextState(ji) - state(ji);
                // double dx_i = nextState(ii + 6);
                // double dx_j = nextState(ji + 6);
                DX(j, i) = dx_i;
                DY(j, i) = dx_j;


                double varnom = 0.0;
                disney::simulation::SimGaussianProcess* sgp = dynamic_cast<disney::simulation::SimGaussianProcess*>(s);
                if (sgp) {
                    varnom = sgp->gaussianProcess()->varianceOfLastPrediction().norm();
                    // cout << "|var| = " << varnom << endl;
                }
                V(j, i) = varnom;

                // if ((i == 25 && j == 25)) {
                //     LOG(INFO) << "-- " << loop << "-- " << i << " " << j;
                //     LOG(INFO) << utils::V2S(state, 4);
                //     LOG(INFO) << utils::V2S(stableState, 4);
                //     LOG(INFO) << utils::V2S(nextState, 4);
                //     // LOG(INFO) << "|var| = " << varnom;
                // }

            }
        }

        if (loop == 0) {
            DX0 = DX;
            DY0 = DY;
        } else {
            double cnt_error = 0.0;
            double sum_error = 0.0;
            for (int i = 0; i < DX.rows(); i++) {
                for (int j = 0; j < DX.cols(); j++) {
                    double dx0 = DX0(i, j);
                    double dy0 = DY0(i, j);
                    double dx1 = DX(i, j);
                    double dy1 = DY(i, j);

                    double error = (dx0 - dx1) * (dx0 - dx1) + (dy0 - dy1) * (dy0 - dy1);
                    sum_error += error;
                    cnt_error += 1.0;
                }
            }
            double avg_error = sum_error / cnt_error;
            LOG(INFO) << "Average of errors = " << avg_error << " at the loop " << loop;
        }

        save(X, "vectorfield_X.txt");
        save(Y, "vectorfield_Y.txt");
        save(DX, "vectorfield_DX.txt");
        save(DY, "vectorfield_DY.txt");
        save(V, "vectorfield_V.txt");

        std::stringstream sout;
        sout << "octave --silent --eval \" ";
        sout << "X  = load('vectorfield_X.txt');";
        sout << "Y  = load('vectorfield_Y.txt');";
        sout << "DX = load('vectorfield_DX.txt');";
        sout << "DY = load('vectorfield_DY.txt');";
        sout << "V  = load('vectorfield_V.txt');";
        sout << "quiver(X, Y, DX, DY, scale=10.0);";
        sout << "axis([" << MINX - MARGIN << " " << MAXX + MARGIN << " "
             << MINY - MARGIN << " " << MAXY + MARGIN << "]);";
        sout << "print -dpng 'plot" << loop << ".png';";
        sout << "contour(X, Y, V, [0.0000001, 0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 1.0]);";
        sout << "colorbar;";
        sout << "axis([" << MINX - MARGIN << " " << MAXX + MARGIN << " "
             << MINY - MARGIN << " " << MAXY + MARGIN << "]);";
        sout << "print -dpng 'var" << loop << ".png';";
        sout << "\" ";

        LOG(INFO) << "cmd = " << endl << sout.str();
        int ret = system(sout.str().c_str());
        LOG(INFO) << "ret = " << ret;
        ret = system("rm vectorfield_*.txt");
        LOG(INFO) << "rm vectorfield_*.txt : " << ret;
        
    }

    
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void compareBox2DandMath(Application* app) {
    LOG(INFO) << FUNCTION_NAME();

    disney::simulation::Manager* manager = app->manager();
    int NS = manager->numSimulators();
    LOG(INFO) << "# of simulators = " << NS;
    CHECK_LT(0, NS);

    simulation::Simulator* s0 = manager->findSimulator(SIMTYPE_BOX2D);
    simulation::Simulator* s1 = manager->findSimulator(SIMTYPE_MATHCALBONGO);

    Eigen::VectorXd state = manager->simulator(0)->state();
    Eigen::VectorXd torque = Eigen::VectorXd::Zero(4);
    // torque << 20, 20, -20, -20;
    // torque << -20, -20, 20, 20;
    torque << 0, 0, -0, -0;
    LOG(INFO) << "state = " << utils::V2S(state, 4);
    LOG(INFO) << "torque = " << utils::V2S(torque, 4);
    std::vector<Eigen::VectorXd> stableStates;

    Eigen::MatrixXd DX0;
    Eigen::MatrixXd DY0;


    int iN = 3;
    int jN = iN;
    double MINX = -0.05;
    double MAXX = 0.05;
    double MINY = -0.05;
    double MAXY = 0.05;
        
    for (int i = 0; i < iN; i++) {
        int ii = 1;
        double xi = interpolatedValue(i, iN, MINX, MAXX);
        state(ii) =  xi;
        // state(0)  = -xi;

        for (int j = 0; j < jN; j++) {
            int ji = 2;
            double xj = interpolatedValue(j, jN, MINY, MAXY);
            state(ji) = xj;
            state(3) = xj;
            state(4) = 0.5 * PI - xj;
            state(5) = -0.5 * PI - xj;

            s0->setState(state);
            LOG(INFO) << endl;
            LOG(INFO) << "Begin " << i << " " << j;
            LOG(INFO) << "initial = " << utils::V2S(state, 4);

            // Copy the s0 to s1
            s1->setState(s0->state());

            for (int k = 0; k < 5; k++) {
                Eigen::VectorXd prevState = s0->state();
                
                // Step forward the simulator 0
                s0->setTorque(torque);
                s0->integrate();
                Eigen::VectorXd ns0 = s0->state();
                // Step forward the simulator 1
                s1->setTorque(torque);
                s1->integrate();
                Eigen::VectorXd ns1 = s1->state();
                LOG(INFO) << "Step " << k;
                // LOG(INFO) << "P : " << utils::V2S(prevState, 6);
                LOG(INFO) << "dB : " << utils::V2S(ns0, 6);
                LOG(INFO) << "dM : " << utils::V2S(ns1, 6);
                LOG(INFO) << "diff = "
                          << (ns0 - ns1).head(6).norm() << " "
                          << (ns0 - ns1).tail(6).norm() ;
            }

        }
    }
    
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void testVectorField3D(Application* app) {
    LOG(INFO) << FUNCTION_NAME();

    disney::simulation::Manager* manager = app->manager();
    simulation::Simulator* s = manager->findSimulator(SIMTYPE_GAUSSIANPROCESS);
    CHECK_NOTNULL(s);
    simulation::SimGaussianProcess* sgp = dynamic_cast<simulation::SimGaussianProcess*>(s);
    CHECK_NOTNULL(sgp);
    sgp->testVectorField3D();
}

} // namespace app
} // namespace disney


