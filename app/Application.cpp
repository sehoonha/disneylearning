/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Application.h"

#include <iomanip>
#include <fstream>
#include <algorithm>
#include "utils/CppCommon.h"
#include "utils/Option.h"
#include "utils/LoadOpengl.h"
#include "utils/Misc.h"
#include "simulation/Simulator.h"
#include "simulation/SimBox2D.h"
#include "simulation/SimMathcalBongo.h"
#include "simulation/SimGaussianProcess.h"
#include "simulation/Manager.h"
#include "simulation/Evaluator.h"
#include "learning/Policy.h"
#include "learning/PolicyFeedback.h"
#include "learning/PolicyPlayback.h"
#include "learning/LearningAlgorithm.h"
#include "learning/LearningPolicyCMASearch.h"
#include "learning/LearningGPSimSearch.h"
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
    , MEMBER_INIT(maxSimLoop, 0)
{
    // test();
}

Application::~Application() {
}


void Application::init() {
    // Initialize all simulators
    set_manager( new simulation::Manager() );
    manager()->init();

    // Load all
    loadAllPolicies();

    CHECK_NOTNULL(policy());

    // Initialize evaluators to all simulators
    FOREACH(simulation::Simulator* sim, manager()->allExistingSimulators()) {
        if (sim->isReserved() == false) {
            sim->set_policy( policy() );
        } else {
            sim->set_policy( policy()->duplicate() );
        }
        sim->set_eval( new simulation::Evaluator() );
    }

    // Initialize learning policy
    // set_learning( new learning::LearningPolicyCMASearch() );
    set_learning( new learning::LearningGPSimSearch() );
    learning()->init();

    // Miscellaneous
    set_maxSimLoop( utils::Option::read("simulation.eval.maxSimLoop").toInt() );
    LOG(INFO) << "Application.maxSimLoop() = " << this->maxSimLoop();
    
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Application::render(bool overlay) {
    int count = 0;
    int num   = manager()->allSimulators().size();
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        glPushMatrix();
        double SCALE_RENDER = 1.0;
        double SCALE_TEXT   = 1.0;
        double TRANSLATE_TEXT_Y = 0.0;
        if (overlay == false) {
            if (num == 2) {
                switch(count) {
                case 0: glTranslated(-1.0, 0.0, 0.0); break;
                case 1: glTranslated( 1.0, 0.0, 0.0); break;
                }
            } else if (num == 3) {
                switch(count) {
                case 0: glTranslated(-1.5, 0.0, 0.0); break;
                case 1: glTranslated( 0.0, 0.0, 0.0); break;
                case 2: glTranslated( 1.5, 0.0, 0.0); break;
                }
                SCALE_RENDER     = 0.9;
                SCALE_TEXT       = 0.7;
                TRANSLATE_TEXT_Y = 0.3;
            }

            // if (count == 0) {
            //     glTranslated(-1.0, 0.0, 0.0);
            // } else if (count == 1) {
            //     glTranslated(1.0, 0.0, 0.0);
            // }
        }
        glPushMatrix();
        glScaled(SCALE_RENDER, SCALE_RENDER, SCALE_RENDER);
        sim->render();
        glPopMatrix();

        glPushMatrix();
        glTranslated(0, TRANSLATE_TEXT_Y, 0);
        glScaled(SCALE_TEXT, SCALE_TEXT, SCALE_TEXT);
        sim->renderInfo();
        glPopMatrix();

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
    if (policy()) {
        policy()->step();
    }
}

void Application::reset() {
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        sim->reset();
    }
    if (policy()) {
        policy()->reset();
    }
}

void Application::train() {
    // learning()->train(policy(), manager()->simulator(1));
    learning()->train(
        manager(),
        policy(),
        manager()->findSimulator(SIMTYPE_BOX2D),
        manager()->findSimulator(SIMTYPE_GAUSSIANPROCESS)
        );
}

void Application::togglePause() {
    learning()->togglePause();
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

void Application::loadHistory(const char* const filename) {
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        sim->loadHistoryFromFile(filename);
    }
}


std::vector<std::string> Application::allSimulatorNames() {
    std::vector<std::string> ret;
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        ret.push_back( sim->type() );
    }
    return ret;
}

void Application::collectData(const char* const _type) {
    simulation::Simulator* sim = manager()->findSimulator(_type);
    // simulation::Simulator* sim = manager()->simulator(1);
    CHECK_NOTNULL(sim);
    LOG(INFO) << "Found a simulator: " << sim->type();
    LOG(INFO) << "Policy: " << sim->policy()->name();
    // std::string filename = "data_math.csv";
    std::string filename
        = std::string("data_") + std::string(_type)
        + std::string("_") + sim->policy()->name() + ".csv";
    LOG(INFO) << "collect data into file " << filename;

    sim->saveHistoryToFile(filename.c_str());

    // std::ofstream fout(filename.c_str(), std::ios::app);
    // fout << std::setprecision(12) << std::fixed;
    // for (int i = 0; i < sim->numHistories(); i++) {
    //     const simulation::SimulatorHistory& h = sim->history(i);
    //     for (int j = 0; j < h.state.size(); j++) {
    //         fout << h.state(j) << " ";
    //     }
    //     fout << "  ";
    //     for (int j = 0; j < h.torque.size(); j++) {
    //         fout << h.torque(j) << " ";
    //     }
    //     fout << endl;
    // }
    // LOG(INFO) << "collect " << sim->numHistories() << " data successfully.";
    // fout.close();
}

void Application::consumeData() {
    simulation::Simulator* sim = manager()->findSimulator(SIMTYPE_GAUSSIANPROCESS);
    CHECK_NOTNULL(sim);
    simulation::SimGaussianProcess* simgp = dynamic_cast<simulation::SimGaussianProcess*>(sim);
    CHECK_NOTNULL(simgp);
    simgp->train();
}

void Application::optimizeGP() {
    simulation::Simulator* sim = manager()->findSimulator(SIMTYPE_GAUSSIANPROCESS);
    CHECK_NOTNULL(sim);
    simulation::SimGaussianProcess* simgp = dynamic_cast<simulation::SimGaussianProcess*>(sim);
    CHECK_NOTNULL(simgp);
    simgp->optimize();
}


// Managing policies
void Application::loadAllPolicies() {
    LOG(INFO) << FUNCTION_NAME();
    FOREACH(const utils::OptionItem& opt, utils::Option::readAll("simulation.policy")) {
        // Initialize a policy
        std::string policy_type = opt.attrString("type");
        LOG(INFO) << endl;
        LOG(INFO) << "policy.type = " << policy_type;

        learning::Policy* current = NULL;
        
        // Case 1. Feedback policy
        if (policy_type == "Feedback") {
            LOG(INFO) << "loading a feedback policy";
            learning::PolicyFeedback* p = new learning::PolicyFeedback();
            p->init();
            current = p;

            std::vector<double> params_v = opt.attrVectorDouble("params");
            Eigen::Map<Eigen::VectorXd> params(params_v.data(), params_v.size());
            LOG(INFO) << "policy.params = " << utils::V2S_SHORT(params);
            p->setParams(params);
        }
        // Case 2. Playback policy
        else if (policy_type == "Playback") { 
            LOG(INFO) << "loading a playback policy";
            learning::PolicyPlayback* p = new learning::PolicyPlayback();
            p->init();
            current = p;

            std::string filename = opt.attrString("filename");
            int n = opt.attrDouble("n");
            int m = opt.attrDouble("m");
            LOG(INFO) << "policy.filename = " << filename;
            LOG(INFO) << "policy.dimensions(n, m) = " << n << " " << m;
            p->load(filename.c_str(), n, m);
            // set_policy( p );
        } else {
            LOG(WARNING) << "we do not have a policy... type = " << policy_type;
        }

        // Follow-up
        if (opt.hasAttr("name")) {
            std::string name = opt.attrString("name");
            current->setName(name);
            LOG(INFO) << "policy.name = " << current->name();
        }
        if (opt.hasAttr("controlStep")) {
            int controlStep = opt.attrInt("controlStep");
            current->setControlStep(controlStep);
            LOG(INFO) << "policy.controlStep = " << controlStep;
        }

        if (opt.hasAttr("select") && opt.attrBool("select")) {
            set_policy( current );
            LOG(INFO) << "Select the current policy!!! ";

        }

        mPolicies.push_back(current);
    }
    LOG(INFO) << FUNCTION_NAME() << " OK";
}


std::string Application::nameOfPolicy(int index) {
    return mPolicies[index]->name();
}

void Application::selectPolicy(int index) {
    learning::Policy* p = mPolicies[index];
    set_policy(p);
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        sim->set_policy( policy() );
    }
    LOG(INFO) << FUNCTION_NAME() << " OK : " << p->name();
}

void Application::selectPolicy(const char* const _name) {
    std::string name(_name);
    FOREACH(learning::Policy* p, mPolicies) {
        if (p->name() == name) {
            set_policy(p);
            FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
                sim->set_policy( policy() );
            }
            LOG(INFO) << FUNCTION_NAME() << " OK : " << p->name();
            return;
        }
    }
    LOG(WARNING) << FUNCTION_NAME() << " cannot find the policy " << name;
}

std::string Application::statusMessage() {
    std::stringstream sout;
    sout << policy()->name() << " : params = " <<  utils::V2S(policy()->params());
    return sout.str();
}

// class Application ends
////////////////////////////////////////////////////////////


} // namespace app
} // namespace disney


