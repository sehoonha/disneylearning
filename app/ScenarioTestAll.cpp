/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "ScenarioTestAll.h"

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
    LOG(INFO) << "Final result" << endl << R;

}

} // namespace app
} // namespace disney


