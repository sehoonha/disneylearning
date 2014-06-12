/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef APP_APPLICATION_H
#define APP_APPLICATION_H

#include "utils/HppCommon.h"

namespace disney {
namespace simulation {
class Simulator;
class Manager;
} // namespace simulation
} // namespace disney

namespace disney {
namespace app {

class Application {
public:
    Application();
    virtual ~Application();

    void init();
    void render();
    void step();
    void reset();
    int numMaximumHistory() const;
    void updateToHistory(int index) const;
protected:
    MEMBER_PTR(simulation::Manager*, manager);
}; // class Application

} // namespace app
} // namespace disney

#endif // #ifndef APP_APPLICATION_H

