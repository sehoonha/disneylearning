/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "LearningGPSimSearch.h"


#include <iomanip>
#include <boost/thread.hpp>

#include "utils/Option.h"
#include "utils/CppCommon.h"
#include "simulation/Simulator.h"
#include "simulation/SimGaussianProcess.h"
#include "simulation/Evaluator.h"
#include "learning/Policy.h"

////////////////////////////////////////////////////////////
// Shark Library
// -- Neural Network 
#include<shark/Models/FFNet.h> // The feed forward neural network
#include<shark/Algorithms/GradientDescent/Rprop.h> // Resilient propagation as optimizer
#include<shark/ObjectiveFunctions/Loss/NegativeClassificationLogLikelihood.h> // Training Loss 
#include<shark/ObjectiveFunctions/ErrorFunction.h> // Error func to connect data model and loss
#include<shark/ObjectiveFunctions/Loss/ZeroOneLoss.h> // Loss for classification
// -- CMA-ES
#include <shark/Algorithms/DirectSearch/CMA.h> // Implementation of the CMA-ES
#include <shark/Algorithms/DirectSearch/ElitistCMA.h>
#include <shark/ObjectiveFunctions/Benchmarks/Benchmarks.h> // Access to benchmark functions
////////////////////////////////////////////////////////////


namespace disney {
namespace learning {


////////////////////////////////////////////////////////////
// struct LearningGPSimSearchImp implementation
struct LearningGPSimSearchImp {
    std::vector<simulation::SimulatorHistory> data;
    learning::Policy* policy;
    simulation::Simulator* s0;
    simulation::SimGaussianProcess* s1;
    int evalCnt0;
    int evalCnt1;

    int maxSimLoop;
    int maxInnerLoop;
    int maxInnerNoUpdateLoop;
    int maxOuterLoop;
    double goodValue;

    LearningGPSimSearchImp();
    
    void collectSim0Data();
    double evaluateSim0();
    double evaluateSim1();
    void learnDynamicsInSim1();
    void optimizePolicyInSim1(int outerLoop); // CMA Optimization

    bool flagPause;
private:
    double evaluate(simulation::Simulator* s);
};

LearningGPSimSearchImp::LearningGPSimSearchImp()
    : policy(NULL)
    , s0(NULL)
    , s1(NULL)
    , evalCnt0(0)
    , evalCnt1(0)
    , flagPause(false)
{
    data.clear();
    this->maxSimLoop   = utils::Option::read("simulation.eval.maxSimLoop").toInt();
    this->maxInnerLoop = utils::Option::read("simulation.eval.maxInnerLoop").toInt();
    this->maxInnerNoUpdateLoop = utils::Option::read("simulation.eval.maxInnerNoUpdateLoop").toInt();
    this->maxOuterLoop = utils::Option::read("simulation.eval.maxOuterLoop").toInt();
    this->goodValue    = utils::Option::read("simulation.eval.goodValue").toDouble();

    LOG(INFO) << "LearningGPSimSearchImp.maxSimLoop = " << maxSimLoop;
    LOG(INFO) << "LearningGPSimSearchImp.maxInnerLoop = " << maxInnerLoop;
    LOG(INFO) << "LearningGPSimSearchImp.maxInnerNoUpdateLoop = " << maxInnerNoUpdateLoop;
    LOG(INFO) << "LearningGPSimSearchImp.maxOuterLoop = " << maxOuterLoop;

}

void LearningGPSimSearchImp::collectSim0Data() {
    for (int i = 0; i < s0->numHistories(); i++) {
        simulation::SimulatorHistory h = s0->history(i);
        data.push_back(h);
    }
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

double LearningGPSimSearchImp::evaluate(simulation::Simulator* s) {
    s->reset();
    for (int i = 0; i < maxSimLoop; i++) {
        s->step();
    }
    double value = s->eval()->cost();
    return value;
}

double LearningGPSimSearchImp::evaluateSim0() {
    CHECK_NOTNULL(s0);
    evalCnt0++;
    return evaluate(s0);
}

double LearningGPSimSearchImp::evaluateSim1() {
    CHECK_NOTNULL(s1);
    evalCnt1++;
    return evaluate(s1);
}

void LearningGPSimSearchImp::learnDynamicsInSim1() {
    std::vector<Eigen::VectorXd> states;
    std::vector<Eigen::VectorXd> torques;
    for (int i = 0; i < data.size(); i++) {
        simulation::SimulatorHistory& h = data[i];        
        states.push_back(h.state);
        torques.push_back(h.torque);
    }
    s1->train(states, torques);
    s1->optimize();
    LOG(INFO) << FUNCTION_NAME() << " OK";

}


// struct LearningGPSimSearchImp ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// Policy Evaluation Function
#define MIN_PARAM -1.0
#define MAX_PARAM 1.0
struct GPPolicyEvaluation : public shark::SingleObjectiveFunction {

    GPPolicyEvaluation(learning::LearningGPSimSearchImp* _imp)
        : mImp(_imp)
        {
            m_features |= CAN_PROPOSE_STARTING_POINT;
        }

    learning::Policy* policy() const { return mImp->policy; }
    
    std::string name() const
        { return "GPPolicyEvaluation"; }

    std::size_t numberOfVariables()const{
        return policy()->numDimParams();
    }

    bool hasScalableDimensionality()const{
        return false;
    }

    void proposeStartingPoint(SearchPointType &x) const {
        Eigen::VectorXd p = policy()->params();
        x.resize(numberOfVariables());
        for (unsigned int i = 0; i < x.size(); i++) {
            x(i) = p(i);
            // x(i) = shark::Rng::uni(MIN_PARAM, MAX_PARAM);
        }
    }

    double eval(const SearchPointType &p) const {
        m_evaluationCounter++;

        // Setup the parameters
        Eigen::VectorXd params( p. size() );
        for (int i = 0; i < p.size(); i++) params(i) = p(i);

        policy()->setParams(params);
        double value = mImp->evaluateSim1();
        LOG(INFO) << "# " << m_evaluationCounter << " : " << params.transpose()
                  << " -> " << value;
        return value;
    }
private:
    learning::LearningGPSimSearchImp* mImp;
};
////////////////////////////////////////////////////////////


// CMA Optimization
void LearningGPSimSearchImp::optimizePolicyInSim1(int outerLoop) {
    LOG(INFO) << FUNCTION_NAME();

    shark::Rng::seed( (unsigned int) time (NULL) );
    GPPolicyEvaluation prob(this);

    // shark::ElitistCMA cma;
    // cma.init(prob);

    shark::CMA cma;
    shark::RealVector starting(prob.numberOfVariables());
    cma.init( prob, starting, 16, 8, 1000.0 );



    double bestValue = 10e10;
    Eigen::VectorXd bestParams;
    int loopCount = 0;
    int noUpdateCount = 0;

    
    do {
        LOG(INFO) << "==== Loop " << loopCount << " in " << outerLoop << " ====";
        cma.step( prob );
        LOG(INFO) << endl;
        LOG(INFO) << prob.evaluationCounter() << " " << cma.solution().value << " " << cma.solution().point << " " << cma.sigma();

        // Update the params;
        shark::RealVector parameters = cma.solution().point;
        Eigen::VectorXd params(parameters.size());
        for (int i = 0; i < params.size(); i++) {
            params(i) = parameters(i);
        }
        policy->setParams(params);

        // Am I improving?
        double v = cma.solution().value;
        if (v < bestValue) {
            noUpdateCount = 0;
            bestValue = v;
            bestParams = params;
        } else {
            noUpdateCount++;
        }
        // hmm...for checking..
        policy->setParams(bestParams);
        
        

        LOG(INFO) << "Summarize the inner loop : ";
        LOG(INFO) << "Best value = " << bestValue << " (noUpdateCount : "<< noUpdateCount << " / "
                  << maxInnerNoUpdateLoop << ")";
        LOG(INFO) << "Best params = " << bestParams.transpose();
        LOG(INFO) << endl;
        if (noUpdateCount >= maxInnerNoUpdateLoop) {
            LOG(INFO) << "Exit the inner loop optimization (CMA) because it is not improving";
            break;
        }


        loopCount++;

        while (flagPause ) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            LOG_EVERY_N(INFO, 100)<< "sleeping... ";
        };

    } while(cma.solution().value > this->goodValue && loopCount < this->maxInnerLoop);
    

    LOG(INFO) << endl << endl;
    LOG(INFO) << "Set the policy current best parameter";
    LOG(INFO) << "Best value  = " << bestValue;
    LOG(INFO) << "Best params = " << bestParams.transpose();
    policy->setParams(bestParams);
    LOG(INFO) << endl << endl;

    LOG(INFO) << FUNCTION_NAME() << " OK";
}


////////////////////////////////////////////////////////////
// thread worker function implementation
void worker(LearningGPSimSearchImp* imp) {
    LOG(INFO) << FUNCTION_NAME() << " begins";

    LOG(INFO) << "Initial optimization";
    imp->optimizePolicyInSim1(-1);

    // 
    for (int loop = 0; loop < imp->maxOuterLoop; loop++) {
        LOG(INFO) << "================== Outer Loop " << loop << " =====================";
        LOG(INFO) << endl;
        LOG(INFO) << "... start to evaluate in the real world";
        double v = imp->evaluateSim0();
        LOG(INFO) << "... finished to evaluate in the real world";
        LOG(INFO) << "result: " << v;

        if (v < imp->goodValue) {
            LOG(INFO) << "termintate. " << v << " is less than threshold " << imp->goodValue;
            break;
        }
        imp->collectSim0Data();
        imp->learnDynamicsInSim1();
        imp->optimizePolicyInSim1(loop);
    }

    
    LOG(INFO) << FUNCTION_NAME() << " OK";

}

// thread worker function ends
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
// class LearningGPSimSearch implementation
LearningGPSimSearch::LearningGPSimSearch()
    : imp(NULL)
{
}


LearningGPSimSearch::~LearningGPSimSearch() {
    if (imp) { delete imp; imp = NULL; }
}

void LearningGPSimSearch::init() {
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void LearningGPSimSearch::train(learning::Policy* _policy,
                                simulation::Simulator* _sim0,
                                simulation::Simulator* _sim1) {
    using namespace simulation;
    LOG(INFO) << FUNCTION_NAME();
    // Fetch simulators
    Simulator* s0 = _sim0; // real environment
    SimGaussianProcess* s1 = dynamic_cast<SimGaussianProcess*>(_sim1); // virtual environment
    CHECK_NOTNULL(s0);
    CHECK_NOTNULL(s1);
    LOG(INFO) << "Simulator 0: " << s0->type();
    LOG(INFO) << "Simulator 1: " << s1->type();

    // Initialize the imp structure
    if (imp) { delete imp; imp = NULL; }
    imp = new LearningGPSimSearchImp;
    imp->s0 = s0;
    imp->s1 = s1;
    imp->policy = _policy;
    LOG(INFO) << "Implementation structure is initialized OK";

    LOG(INFO) << "launch the new worker thread...";
    boost::thread t(&worker, imp);

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void LearningGPSimSearch::togglePause() {
    if (imp) {
        imp->flagPause = !(imp->flagPause);
        LOG(INFO) << FUNCTION_NAME() << " : set the pause flag as " << imp->flagPause;
    } else {
        LOG(INFO) << FUNCTION_NAME() << " : no running polich search..";
    }
}


// class LearningGPSimSearch ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disney


