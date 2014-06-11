#ifndef BOX2DSIMULATION_H
#define BOX2DSIMULATION_H

#include <Eigen/Dense>
#include <vector>
#include "hppcommon.h"

namespace disneysimple {
namespace sim {

struct Box2dSimulationImp;

class Box2dSimulation {
public:
    Box2dSimulation();
    virtual ~Box2dSimulation();

    void init();
    void step();
    void render();

protected:
    Box2dSimulationImp* imp;
    
}; // class Box2dSimulation

} // namespace sim
} // namespace disneysimple

#endif // #ifndef BOX2DSIMULATION_H

