/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "ScenarioTestAll.h"

#include <fstream>
#include <iomanip>
#include <boost/thread.hpp>

#include "utils/CppCommon.h"
#include "utils/Misc.h"
#include "simulation/Manager.h"
#include "simulation/Simulator.h"
#include "simulation/Evaluator.h"

#include "Application.h"

namespace disney {
namespace app {

void ScenarioTestAll(Application* app) {
    disney::simulation::Manager* manager = app->manager();

    int NP = app->numPolicies();
    int NS = manager->numSimulators();
    
    LOG(INFO) << "# of policies = " << NP;
    LOG(INFO) << "# of simulators = " << NS;

    Eigen::MatrixXd R(NP, NS);

    int maxSimLoop = app->maxSimLoop();
    LOG(INFO) << "maxSimLoop = " << maxSimLoop;

    for (int i = 0; i < NP; i++) {
        LOG(INFO) << "Evaluate policy " << i << " (" << app->nameOfPolicy(i) << ")";
        app->selectPolicy(i);
        app->reset();
        for (int j = 0; j < maxSimLoop; j++) {
            app->step();
        }

        for (int j = 0; j < NS; j++) {
            simulation::Simulator* s = manager->simulator(j);
            double value = s->eval()->cost();
            R(i, j) = value;
        }
        LOG(INFO) << "Cost at row " << i << " : " << utils::V2S_SHORT(R.row(i));
    }

    // LOG(INFO) << "Final result" << endl << R;
    LOG(INFO) << "Final result";
    std::ofstream fout("testall.csv");
    cout << std::fixed << std::setprecision(4);
    fout << std::fixed << std::setprecision(4);
    cout << "-";
    fout << "-";
    for (int j = 0; j < R.cols(); j++) {
        simulation::Simulator* s = manager->simulator(j);
        cout << ", " << s->type();
        fout << ", " << s->type();
    }
    cout << endl;
    fout << endl;

    for (int i = 0; i < R.rows(); i++) {
        cout << app->nameOfPolicy(i);
        fout << app->nameOfPolicy(i);
        for (int j = 0; j < R.cols(); j++) {
            cout << ", " << R(i, j);
            fout << ", " << R(i, j);
        }
        cout << endl;
        fout << endl;
    }
    fout.close();
}

} // namespace app
} // namespace disney


