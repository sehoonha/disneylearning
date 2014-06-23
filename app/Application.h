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
// class Evaluator;
} // namespace simulation
namespace learning {
class Policy;
class LearningAlgorithm;
} // namespace learning
} // namespace disney

namespace disney {
namespace app {

class Application {
public:
    Application();
    virtual ~Application();

    void init();
    void render(bool overlay = false);
    void step();
    void reset();
    void train();
    int numMaximumHistory() const;
    void updateToHistory(int index) const;

    void collectData();
    void consumeData();
protected:
    MEMBER_PTR(simulation::Manager*, manager);
    // MEMBER_PTR(simulation::Evaluator*, eval);
    MEMBER_PTR(learning::Policy*, policy);
    MEMBER_PTR(learning::LearningAlgorithm*, learning);

}; // class Application

} // namespace app
} // namespace disney

#endif // #ifndef APP_APPLICATION_H

