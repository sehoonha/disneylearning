/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Application.h"

#include <algorithm>
#include "utils/CppCommon.h"
#include "utils/LoadOpengl.h"
#include "simulation/Simulator.h"
#include "simulation/Manager.h"
#include "simulation/Evaluator.h"
#include "learning/Policy.h"
#include "learning/PolicyFeedback.h"
#include "learning/LearningAlgorithm.h"
#include "learning/LearningPolicyCMASearch.h"
#include "learning/GaussianProcess.h"



namespace disney {
namespace app {

double g0(double x0, double x1) {
    return 0.2 * sin(3.0 * x0) + 0.2 * x1 - 0.1 * x0;
}

double g1(double x0, double x1) {
    return -0.3 * (x1 - x0) * (x1 - x0) + 0.1 + x0 * (x1 - 2.0);
}

double g0_noise(double x0, double x1) {
    return g0(x0, x1) + 0.01 * (drand48() - 0.5);
}

double g1_noise(double x0, double x1) {
    return g1(x0, x1) + 0.01 * (drand48() - 0.5);
}

void test() {
    srand48( (unsigned int) time (NULL) );
    int n = 2000;
    int d = 2;
    Eigen::MatrixXd X(n, d);
    Eigen::MatrixXd Y(n, d);

    for (int i = 0; i < n; i++) {
        X(i, 0) = drand48() * 2.0 - 1.0;
        X(i, 1) = drand48() * 2.0 - 1.0;

        Y(i, 0) = g0_noise( X(i, 0), X(i, 1) );
        Y(i, 1) = g1_noise( X(i, 0), X(i, 1) );
    }

    learning::GaussianProcess gp;
    gp.createModel(X, Y);
    // gp.optimize();
    
    int m = 10;
    double sum_error = 0.0;
    for (int i = 0; i < m; i++) {
        Eigen::VectorXd x(d);
        x(0) = drand48() * 2.0 - 1.0;
        x(1) = drand48() * 2.0 - 1.0;
        Eigen::VectorXd y = gp.predict(x);
        Eigen::VectorXd ybar(d);
        ybar(0) = g0( x(0), x(1) );
        ybar(1) = g1( x(0), x(1) );
        double error = (y - ybar).squaredNorm();
        sum_error += error;
        cout << i << " : "
             << x.transpose() << " --> " << y.transpose() << " ("
             << ybar.transpose() << ", err = " << error << ")"
             << endl;
        
    }
    double avg_error = sum_error / m;
    LOG(INFO) << "sum_error = " << sum_error;
    LOG(INFO) << "avg_error = " << avg_error;
    CHECK_LT(avg_error, 0.01);
}

////////////////////////////////////////////////////////////
// class Application implementation
Application::Application()
    : MEMBER_INIT_NULL(manager)
    // , MEMBER_INIT_NULL(eval)
    , MEMBER_INIT_NULL(policy)
    , MEMBER_INIT_NULL(learning)
{
    test();
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
    learning()->train(policy(), manager()->simulator(0));
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

// class Application ends
////////////////////////////////////////////////////////////


} // namespace app
} // namespace disney


