#ifndef DISNEYSIMPLE_RLEVOLUTION_H
#define DISNEYSIMPLE_RLEVOLUTION_H

#include "hppcommon.h"
#include <Eigen/Dense>

namespace disneysimple {
namespace sim {
class Simulation;
} // namespace sim
} // namespace disneysimple


namespace disneysimple {
namespace learning {

struct RLEvolutionImp;

class RLEvolution {
public:
    RLEvolution();
    virtual ~RLEvolution();

    // For training
    void train(sim::Simulation* sim);
    void save();
    void load(const char* const filename);

    // For using
    Eigen::VectorXd control(const Eigen::VectorXd& x);
    
protected:
    RLEvolutionImp* imp;
}; // class RLEvolution

} // namespace learning
} // namespace disneysimple

#endif // #ifndef DISNEYSIMPLE_RLEVOLUTION_H

