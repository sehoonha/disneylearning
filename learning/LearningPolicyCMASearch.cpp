/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "LearningPolicyCMASearch.h"
#include <iomanip>
#include "utils/CppCommon.h"
#include "simulation/Simulator.h"
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
// Policy Evaluation Function
#define MIN_PARAM -1.0
#define MAX_PARAM 1.0
struct PolicyEvaluation : public shark::SingleObjectiveFunction {

    PolicyEvaluation(learning::Policy* _policy,
                     simulation::Simulator* _sim)
        : mPolicy(_policy)
        , mSim(_sim)
        {
            m_features |= CAN_PROPOSE_STARTING_POINT;
            // m_features |= IS_CONSTRAINED_FEATURE;
            // m_features |= CAN_PROVIDE_CLOSEST_FEASIBLE;
        }
    
    std::string name() const
        { return "PolicyEvaluation"; }

    std::size_t numberOfVariables()const{
        return mPolicy->numDimParams();
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

        mPolicy->setParams(params);

        mSim->reset();
        for (int i = 0; i < 3000; i++) {
            mSim->step();
        }
        double value = mSim->eval()->cost();
        LOG(INFO) << "# " << m_evaluationCounter << " : " << params.transpose()
                  << " -> " << value;
        return value;
    }
private:
    learning::Policy* mPolicy;
    simulation::Simulator* mSim;

};
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
// class LearningPolicyCMASearch implementation

LearningPolicyCMASearch::LearningPolicyCMASearch() {
}

LearningPolicyCMASearch::~LearningPolicyCMASearch() {
}

void LearningPolicyCMASearch::init() {
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void LearningPolicyCMASearch::train(learning::Policy* _policy,
                                    simulation::Simulator* _sim) {
    LOG(INFO) << FUNCTION_NAME();
    LOG(INFO) << "simulator type = " << _sim->type();

    shark::Rng::seed( (unsigned int) time (NULL) );

    PolicyEvaluation prob(_policy, _sim);
    shark::CMA cma;
    // cma.init( prob );
    // cma.setSigma(10.0);
    shark::RealVector starting(prob.numberOfVariables());
    // cma.init( prob, starting, 32, 16, 15.0 );
    // cma.init( prob, starting, 32, 16, 10.0 );
    cma.init( prob, starting, 32, 16, 1000.0 );


    int loopCount = 0;
    do {
        LOG(INFO) << "==== Loop " << loopCount << " ====";
        cma.step( prob );

        // Report information on the optimizer state and the current solution to the console.


        LOG(INFO) << prob.evaluationCounter() << " " << cma.solution().value << " " << cma.solution().point << " " << cma.sigma();

        // Update the params;
        shark::RealVector parameters = cma.solution().point;
        Eigen::VectorXd params(parameters.size());
        for (int i = 0; i < params.size(); i++) {
            params(i) = parameters(i);
        }
        _policy->setParams(params);
        loopCount++;
    } while(cma.solution().value > (0.0) && loopCount < 1000);

}


// class LearningPolicyCMASearch ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disney


