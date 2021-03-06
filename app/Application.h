/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef APP_APPLICATION_H
#define APP_APPLICATION_H

#include <vector>
#include <string>
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
    void train(bool launchThread = true);
    void togglePause();

    int numMaximumHistory() const;
    void updateToHistory(int index) const;
    void loadHistory(const char* const filename);

    std::vector<std::string> allSimulatorNames();
    void collectData(const char* const _type);
    void consumeData();
    void optimizeGP();

    // Managing policies
    void loadAllPolicies();
    int numPolicies() const { return mPolicies.size(); }
    std::string nameOfPolicy(int index);
    void selectPolicy(int index);
    void selectPolicy(const char* const _name);
    std::string statusMessage();
protected:
    MEMBER_PTR(simulation::Manager*, manager);
    // MEMBER_PTR(simulation::Evaluator*, eval);
    MEMBER_PTR(learning::LearningAlgorithm*, learning);
    MEMBER_VAR(int, maxSimLoop);

    MEMBER_PTR(learning::Policy*, policy);
    std::vector<learning::Policy*> mPolicies;
}; // class Application

} // namespace app
} // namespace disney

#endif // #ifndef APP_APPLICATION_H

