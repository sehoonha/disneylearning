/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Application.h"
#include "utils/CppCommon.h"
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
    manager()->simulator(0)->render();
}


// class Application ends
////////////////////////////////////////////////////////////


} // namespace app
} // namespace disney


