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
    void control();
    void step();
    void render();

    Eigen::VectorXd getState();
    void setState(const Eigen::VectorXd& state);
    
    void reset();
    int nStateHistory() const { return mStateHistory.size(); }
    void updateToHistory(int index);
    void updateToLatestHistory() { updateToHistory( nStateHistory() - 1); }

    Eigen::VectorXd getControlState();
    void setControlState(const Eigen::VectorXd& state);
protected:
    Box2dSimulationImp* imp;
    std::vector<Eigen::VectorXd> mStateHistory;
    
}; // class Box2dSimulation

} // namespace sim
} // namespace disneysimple

#endif // #ifndef BOX2DSIMULATION_H

