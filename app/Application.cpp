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

namespace disney {
namespace app {
////////////////////////////////////////////////////////////
// class Application implementation
Application::Application()
    : MEMBER_INIT_NULL(manager)
{
}

Application::~Application() {
}

void Application::init() {
    set_manager( new simulation::Manager() );
    manager()->init();
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Application::render() {
    int count = 0;
    FOREACH(simulation::Simulator* sim, manager()->allSimulators()) {
        glPushMatrix();
        if (count == 0) {
            glTranslated(-1.0, 0.0, 0.0);
        } else {
            glTranslated(1.0, 0.0, 0.0);
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


