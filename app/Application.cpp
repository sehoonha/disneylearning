/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Application.h"

#include <fstream>
#include <algorithm>
#include "utils/CppCommon.h"
#include "utils/LoadOpengl.h"
#include "simulation/Simulator.h"
#include "simulation/SimBox2D.h"
#include "simulation/SimMathcalBongo.h"
#include "simulation/SimGaussianProcess.h"
#include "simulation/Manager.h"
#include "simulation/Evaluator.h"
#include "learning/Policy.h"
#include "learning/PolicyFeedback.h"
#include "learning/LearningAlgorithm.h"
#include "learning/LearningPolicyCMASearch.h"
#include "learning/GaussianProcess.h"



namespace disney {
namespace app {
////////////////////////////////////////////////////////////
// class Application implementation
Application::Application()
    : MEMBER_INIT_NULL(manager)
    // , MEMBER_INIT_NULL(eval)
    , MEMBER_INIT_NULL(policy)
    , MEMBER_INIT_NULL(learning)
{
    // test();
}

Application::~Application() {
}


void Application::init() {
    set_manager( new simulation::Manager() );
    manager()->init();

    // set_eval( new simulation::Evaluator() );
    
    set_policy ( new learning::PolicyFeedback() );
    policy()->init();
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        sim->set_policy( policy() );
        sim->set_eval( new simulation::Evaluator() );
    }

    set_learning( new learning::LearningPolicyCMASearch() );
    learning()->init();

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Application::render(bool overlay) {
    int count = 0;
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        glPushMatrix();
        if (overlay == false) {
            if (count == 0) {
                glTranslated(-1.0, 0.0, 0.0);
            } else {
                glTranslated(1.0, 0.0, 0.0);
            }
        }
        sim->render();
        sim->renderInfo();
        glPopMatrix();

        count++;
    }
}

void Application::step() {
    // for (int i = 0; i < manager()->numSimulators(); i++) {
    //     simulation::Simulator* sim = manager()->simulator(i);
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        sim->step();
    }
}

void Application::reset() {
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        sim->reset();
    }
}

void Application::train() {
    learning()->train(policy(), manager()->simulator(1));
}

int Application::numMaximumHistory() const {
    int ret = 0;
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        ret = std::max(ret, sim->numHistories());
    }
    return ret;
}

void Application::updateToHistory(int index) const {
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        if (index < sim->numHistories()) {
            sim->updateToHistory(index);
        }
    }
}

void Application::collectData() {
    // simulation::Simulator* sim = manager()->availableSimulator(SIMTYPE_BOX2D);
    simulation::Simulator* sim = manager()->simulator(1);
    CHECK_NOTNULL(sim);
    LOG(INFO) << "Found a simulator: " << sim->type();
    // std::string filename = "data_math.csv";
    std::string filename = "data_gp.csv";
    LOG(INFO) << "collect data into file " << filename;

    std::ofstream fout(filename.c_str(), std::ios::app);
    for (int i = 0; i < sim->numHistories(); i++) {
        const simulation::SimulatorHistory& h = sim->history(i);
        for (int j = 0; j < h.state.size(); j++) {
            fout << h.state(j) << " ";
        }
        fout << "  ";
        for (int j = 0; j < h.torque.size(); j++) {
            fout << h.torque(j) << " ";
        }
        fout << endl;
    }
    LOG(INFO) << "collect " << sim->numHistories() << " data successfully.";
    fout.close();
}

void Application::consumeData() {
    simulation::Simulator* sim = manager()->availableSimulator(SIMTYPE_GAUSSIANPROCESS);
    CHECK_NOTNULL(sim);
    simulation::SimGaussianProcess* simgp = dynamic_cast<simulation::SimGaussianProcess*>(sim);
    CHECK_NOTNULL(simgp);
    simgp->train();
}
void Application::optimizeGP() {
    simulation::Simulator* sim = manager()->availableSimulator(SIMTYPE_GAUSSIANPROCESS);
    CHECK_NOTNULL(sim);
    simulation::SimGaussianProcess* simgp = dynamic_cast<simulation::SimGaussianProcess*>(sim);
    CHECK_NOTNULL(simgp);
    simgp->optimize();
}


// class Application ends
////////////////////////////////////////////////////////////


} // namespace app
} // namespace disney


