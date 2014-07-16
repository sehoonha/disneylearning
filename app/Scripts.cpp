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
#include "simulation/Evaluator.h"

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
    Eigen::VectorXd stableState = manager->simulator(0)->state();
    Eigen::VectorXd torque = Eigen::VectorXd::Zero(4);
    LOG(INFO) << "state = " << utils::V2S(state, 4);
    LOG(INFO) << "torque = " << utils::V2S(torque, 4);

    for (int loop = 0; loop < NS; loop++) {
        simulation::Simulator* s = manager->simulator(loop);

        int iN = 21;
        int jN = iN;

        Eigen::VectorXd X(iN);
        Eigen::VectorXd Y(jN);
        Eigen::MatrixXd DX(jN, iN);
        Eigen::MatrixXd DY(jN, iN);
        
        for (int i = 0; i < iN; i++) {
            int ii = 1;
            double xi = interpolatedValue(i, iN, -0.2, 0.2);
            state(ii) = X(i) = xi;

            for (int j = 0; j < jN; j++) {
                int ji = 2;
                double xj = interpolatedValue(j, jN, -0.2, 0.2);
                state(ji) = Y(j) = xj;

                Eigen::VectorXd nextState;
                if (loop == 0) {
                    s->setState(state);
                    s->setTorque(torque);
                    s->integrate();
                    stableState = s->state();
                    s->setTorque(torque);
                    s->integrate();
                    nextState = s->state();
                    
                } else {
                    s->setState(stableState);
                    s->setTorque(torque);
                    s->integrate();
                    nextState = s->state();
                }
                // Eigen::VectorXd nextState = s->state();
                double dx_i = nextState(ii) - stableState(ii);
                double dx_j = nextState(ji) - stableState(ji);
                DX(j, i) = dx_i;
                DY(j, i) = dx_j;
            }
        }

        save(X, "vectorfield_X.txt");
        save(Y, "vectorfield_Y.txt");
        save(DX, "vectorfield_DX.txt");
        save(DY, "vectorfield_DY.txt");

        std::stringstream sout;
        sout << "octave --eval \" ";
        sout << "X = load('vectorfield_X.txt');";
        sout << "Y = load('vectorfield_Y.txt');";
        sout << "DX = load('vectorfield_DX.txt');";
        sout << "DY = load('vectorfield_DY.txt');";
        sout << "quiver(X, Y, DX, DY, scale=10.0);";
        sout << "print -dpng 'plot" << loop << ".png';";
        sout << "\" ";

        LOG(INFO) << "cmd = " << endl << sout.str();
        int ret = system(sout.str().c_str());
        LOG(INFO) << "ret = " << ret;
        ret = system("rm vectorfield_*.txt");
        LOG(INFO) << "rm vectorfield_*.txt : " << ret;
        
    }

    
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

} // namespace app
} // namespace disney


