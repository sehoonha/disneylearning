/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "LearningGPSimSearch.h"

#include <iomanip>
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

    LearningGPSimSearchImp();
    
    void collect();
    double evaluateSim0();
    double evaluateSim1();
    void learnDynamics();
    void optimizePolicyInSim1(); // CMA Optimization

private:
    double evaluate(simulation::Simulator* s);
};

LearningGPSimSearchImp::LearningGPSimSearchImp()
    : policy(NULL)
    , s0(NULL)
    , s1(NULL)
    , evalCnt0(0)
    , evalCnt1(0)
{
}

void LearningGPSimSearchImp::collect() {
}

double LearningGPSimSearchImp::evaluate(simulation::Simulator* s) {
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

void LearningGPSimSearchImp::learnDynamics() {
}

// CMA Optimization
void LearningGPSimSearchImp::optimizePolicyInSim1() {
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
        x.resize(numberOfVariables());
        for (unsigned int i = 0; i < x.size(); i++) {
            x(i) = shark::Rng::uni(MIN_PARAM, MAX_PARAM);
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
    LOG(INFO) << "Implementation structure is initialized OK";
    

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

// class LearningGPSimSearch ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disney


