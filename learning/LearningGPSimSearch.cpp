/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "LearningGPSimSearch.h"


#include <fstream>
#include <iomanip>
#include <boost/thread.hpp>

#include "utils/Option.h"
#include "utils/CppCommon.h"
#include "utils/Misc.h"
#include "simulation/Manager.h"
#include "simulation/Simulator.h"
#include "simulation/SimGaussianProcess.h"
#include "simulation/Evaluator.h"
#include "learning/Policy.h"
#include "learning/GaussianProcess.h"

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
    simulation::Manager* manager;
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

    void collectHistoryData();
    void collectSim0Data();
    double evaluateSim0(const Eigen::VectorXd& params);
    double evaluateSim1(const Eigen::VectorXd& params, int* pOutSimId = NULL);
    void learnDynamicsInSim1();
    void optimizePolicyInSim1(int outerLoop); // CMA Optimization
    void testAllSimulators();

    bool flagPause;
private:
    double evaluate(simulation::Simulator* s);
};

LearningGPSimSearchImp::LearningGPSimSearchImp()
    : manager(NULL)
    , policy(NULL)
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

void LearningGPSimSearchImp::collectHistoryData() {
    int n = s0->numDimState();
    int m = s0->numDimTorque();

    FOREACH(const utils::OptionItem& o, utils::Option::readAll("simulation.gp.data")) {
        const std::string filename = o.attrString("filename");
        LOG(INFO) << "Data filename = [" << filename << "]";
        std::ifstream fin(filename.c_str());

        int loop = 0;
        for (; ; loop++) {
            Eigen::VectorXd currState  = Eigen::VectorXd::Zero(n);
            Eigen::VectorXd currTorque = Eigen::VectorXd::Zero(m);

            for (int i = 0; i < n; i++) {
                fin >> currState(i);
            }
            for (int i = 0; i < m; i++) {
                fin >> currTorque(i);
            }
            if (fin.fail()) {
                LOG(INFO) << "end of data at loop = " << loop;
                break;
            }

            simulation::SimulatorHistory h;
            h.state = currState;
            h.torque = currTorque;
            data.push_back(h);
        }
        fin.close();

        LOG(INFO) << "Data filename = [" << filename << "] : # lines = " << loop;
        LOG(INFO) << "--> # states = " << data.size();
    }
    LOG(INFO) << "Total # histories = " << data.size();
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
    // if (value < 100.0) {
    //     LOG(INFO) << "Save the good history into the snapshot";
    //     LOG(INFO) << "params = " << utils::V2S(s->policy()->params());
    //     s->saveHistoryToFile("good.csv");
    // }
    return value;
}

double LearningGPSimSearchImp::evaluateSim0(const Eigen::VectorXd& params) {
    CHECK_NOTNULL(s0);
    s0->policy()->setParams(params);
    evalCnt0++;
    return evaluate(s0);
}

double LearningGPSimSearchImp::evaluateSim1(const Eigen::VectorXd& params, int* pOutSimId) {
    simulation::SimGaussianProcess* sgp = NULL;
    while(1) {
        simulation::Simulator* s =  manager->unoccupiedSimulator(SIMTYPE_GAUSSIANPROCESS);
        sgp = dynamic_cast<simulation::SimGaussianProcess*>(s);
        if (sgp) {
            break;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    CHECK_NOTNULL(sgp);
    sgp->policy()->setParams(params);
    if (pOutSimId) {
        (*pOutSimId) = sgp->id();
    }
    evalCnt1++;
    double value = evaluate(sgp);
    manager->markSimulatorAsUnoccupied(sgp);
    return value;
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

    CHECK_NOTNULL(manager);
    Eigen::VectorXd hyper = s1->gaussianProcess()->hyperParameters();
    FOREACH(simulation::Simulator* rs, manager->allReservedSimulators()) {
        simulation::SimGaussianProcess* sgp = dynamic_cast<simulation::SimGaussianProcess*>(rs);
        if (sgp == NULL) {
            continue;
        }
        sgp->train(states, torques);
        sgp->gaussianProcess()->setHyperParameters(hyper);
    }

    LOG(INFO) << FUNCTION_NAME() << " OK";

}

void LearningGPSimSearchImp::testAllSimulators() {
    Eigen::VectorXd p = Eigen::VectorXd::Random(5);
    LOG(INFO) << "Random parameter for testing = " << utils::V2S_SHORT(p);
    policy->setParams(p);
    for (int i = 2; i < 7; i++) {
        simulation::Simulator* s = manager->simulator(i);
        double v = evaluate(s);
        LOG(INFO) << "Test Sim " << s->id() << " " << s->type() << " --> " << v;
    }
    LOG(INFO) << FUNCTION_NAME() << " OK";
    exit(0);
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
            mInnerLoopOnSim0 = utils::Option::read("simulation.eval.innerLoopOnSim0").toBool();
            if (mInnerLoopOnSim0) {
                LOG(INFO) << "Inner Loop Optimization is set as Simulation 0 Mode!!!";
            }
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

        // policy()->setParams(params);
        double value = 0.0;
        int id = -1;
        if (mInnerLoopOnSim0) {
            value = mImp->evaluateSim0(params);
        } else {
            value = mImp->evaluateSim1(params, &id);
        }
        LOG(INFO) << "Id(" << id << ")"
                  << "# " << m_evaluationCounter << " : " << utils::V2S_SHORT(params)
                  << " -> " << value;

        
        // double err = (params - mImp->manager->simulator(id)->policy()->params()).squaredNorm();
        // // if (err > 1.0) {
        // LOG(INFO) << "err = " << err << endl << utils::V2S(params) << endl <<  utils::V2S(policy()->params());

        // }
        
        return value;
    }
private:
    learning::LearningGPSimSearchImp* mImp;
    bool mInnerLoopOnSim0;
};
////////////////////////////////////////////////////////////


// CMA Optimization
void LearningGPSimSearchImp::optimizePolicyInSim1(int outerLoop) {
    LOG(INFO) << FUNCTION_NAME();

    shark::Rng::seed( (unsigned int) time (NULL) );
    // shark::Rng::seed( 5558 );
    GPPolicyEvaluation prob(this);

    // shark::ElitistCMA cma;
    // cma.init(prob);

    shark::CMA cma;
    shark::RealVector starting(prob.numberOfVariables());
    bool startFromLast = utils::Option::read("simulation.eval.cma.startFromLast").toBool();
    LOG(INFO) << "startFromLast = " << startFromLast;
    if (startFromLast) {
        LOG(INFO) << "Copy the policy parameter to the start point";
        Eigen::VectorXd policyParams = policy->params();
        for (int i = 0; i < policyParams.size(); i++) {
            starting(i) = policyParams(i);
        }
    }
    int lambda = utils::Option::read("simulation.eval.cma.lambda").toInt();
    int mu = utils::Option::read("simulation.eval.cma.mu").toInt();
    double initialStep = utils::Option::read("simulation.eval.cma.initialStep").toDouble();
    LOG(INFO) << "lambda = " << lambda;
    LOG(INFO) << "mu = " << mu;
    LOG(INFO) << "initialStep = " << initialStep;
    LOG(INFO) << "starting = " << starting;
    cma.init( prob, starting, lambda, mu, initialStep );


    double bestValue = 10e10;
    Eigen::VectorXd bestParams;
    int loopCount = 0;
    int noUpdateCount = 0;

    bool stepParallel = utils::Option::read("simulation.eval.training.stepParallel").toBool();
    LOG(INFO) << "stepParallel = " << stepParallel;
    do {
        LOG(INFO) << "==== Loop " << loopCount << " in " << outerLoop << " ====";

        if (stepParallel) {
            LOG(INFO) << "Take the parallel step";
            cma.stepParallel( prob );
        } else {
            LOG(INFO) << "Take the single core step";
            cma.step( prob );
        }
        LOG(INFO) << endl;
        LOG(INFO) << prob.evaluationCounter() << " " << cma.solution().value << " " << cma.solution().point;
        LOG(INFO) << "mean = " << cma.mean();
        LOG(INFO) << "sigma = " << cma.sigma();
        LOG(INFO) << "cov = " << endl << cma.covarianceMatrix();

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
        LOG(INFO) << "Best params = " << utils::V2S(bestParams);
        LOG(INFO) << endl;
        LOG(INFO) << std::flush;
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
    LOG(INFO) << "Best params = " << utils::V2S(bestParams);
    policy->setParams(bestParams);
    LOG(INFO) << endl << endl;

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void exportIntermediateResult(LearningGPSimSearchImp* imp, int loop) {
    LOG(INFO) << "================ Start to export intermediate result  ============";

    // 0. execute the best parameters in the simulation
    LOG(INFO) << "Evaluate the simulation to get the history";
    int id;
    Eigen::VectorXd bestParams = imp->policy->params();
    LOG(INFO) << "  bestParams = " << utils::V2S(bestParams);
    imp->evaluateSim1(bestParams, &id);

    // 1. Save the simulation history
    std::string filename_sim = (boost::format("intermediate_l%02d_sim.csv") % loop).str();
    LOG(INFO) << "Save the simulation history to " << filename_sim;
    imp->s1->saveHistoryToFile( filename_sim.c_str() );

    // 2. Save the real history
    std::string filename_real = (boost::format("intermediate_l%02d_real.csv") % loop).str();
    LOG(INFO) << "Save the real environment history to " << filename_real;
    imp->s0->saveHistoryToFile( filename_real.c_str() );

    // 3. Save the entire training data
    std::string filename_train = (boost::format("intermediate_l%02d_train.csv") % loop).str();
    LOG(INFO) << "Save the real environment history to " << filename_train;
    std::vector<simulation::SimulatorHistory> backup = imp->s1->allHistories();
    imp->s0->setAllHistories(imp->data);
    imp->s0->saveHistoryToFile( filename_train.c_str() );
    imp->s0->setAllHistories(backup);
    LOG(INFO) << "# histories of s0 (should be short) = " << imp->s0->numHistories();

    // 4. Save the stats
    std::string filename_txt = "intermediate.txt";
    if (loop == -1) {
        LOG(INFO) << "Delete " << filename_txt;
        system("rm intermediate.txt");
    }
    LOG(INFO) << "Save(append) the statistics to " << filename_txt;
    std::ofstream fout(filename_txt.c_str(), std::ios::app);
    if (loop == -1) {
        fout << "iter, num_data, sim, real, params" << endl; 
    }
    fout << loop << ", ";
    fout << imp->data.size() << ", ";
    fout << imp->s0->eval()->cost() << ", ";
    fout << imp->s1->eval()->cost() << ", ";
    fout << utils::V2S(imp->policy->params());
    fout << endl;
    fout.close();

    LOG(INFO) << "================ Finish to export intermediate result  ============";
    
}

////////////////////////////////////////////////////////////
// thread worker function implementation
void worker(LearningGPSimSearchImp* imp) {
    LOG(INFO) << FUNCTION_NAME() << " begins";

    LOG(INFO) << "Initial optimization";

    // {
    //     // Testing code
    //     imp->collectSim0Data();
    //     imp->learnDynamicsInSim1();
    //     imp->testAllSimulators();
    // }
    bool loadDataAtInitialTraining = utils::Option::read("simulation.eval.training.loadDataAtInitialTraining").toBool();
    LOG(INFO) << "loadDataAtInitialTraining = " << loadDataAtInitialTraining;
    if (loadDataAtInitialTraining) {
        LOG(INFO) << "========================================================";
        LOG(INFO) << "================ Start to load initial data ============";
        imp->collectHistoryData();
        imp->learnDynamicsInSim1();
        LOG(INFO) << "================ Finish to load initial data ============";
        LOG(INFO) << "=========================================================";
    } else {
        LOG(INFO) << "=======================================================";
        LOG(INFO) << "================ Skip to load initial data ============";
        LOG(INFO) << "=======================================================";
    }

    // report the simulations
    FOREACH(simulation::Simulator* s, imp->manager->allExistingSimulators()) {
        simulation::SimGaussianProcess* sgp = dynamic_cast<simulation::SimGaussianProcess*>(s);
        if (sgp == NULL) {
            LOG(INFO) << s->type() << "[" << s->id() << "]";
        } else {
            if (sgp->gaussianProcess() == NULL) {
                LOG(INFO) << sgp->type() << "[" << sgp->id() << "] : NO GP";
            } else {
                LOG(INFO) << sgp->type() << "[" << sgp->id() << "] : # data = " << sgp->gaussianProcess()->numData();
            }
        }
            
    }

    imp->optimizePolicyInSim1(-1);

    // 
    for (int loop = 0; loop < imp->maxOuterLoop; loop++) {
        LOG(INFO) << "================== Outer Loop " << loop << " =====================";
        LOG(INFO) << endl;
        LOG(INFO) << "... start to evaluate in the real world";
        LOG(INFO) << "... params = " << utils::V2S(imp->policy->params());
        double v = imp->evaluateSim0(imp->policy->params());
        LOG(INFO) << "... finished to evaluate in the real world";
        LOG(INFO) << "result: " << v;

        // Export some intermediate data 
        exportIntermediateResult(imp, loop);

        if (v < imp->goodValue) {
            LOG(INFO) << "termintate. " << v << " is less than threshold " << imp->goodValue;
            break;
        }
        if (loop + 1 == imp->maxOuterLoop) {
            LOG(INFO) << "termintate. " << v << " because reach the last iteration";
        }
        imp->collectSim0Data();
        imp->learnDynamicsInSim1();
        imp->optimizePolicyInSim1(loop);
    }

    
    LOG(INFO) << FUNCTION_NAME() << " OK";

}

void worker_innerloopsim0(LearningGPSimSearchImp* imp) {
    LOG(INFO) << FUNCTION_NAME() << " begins";
    imp->optimizePolicyInSim1(-1);
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

void LearningGPSimSearch::train(simulation::Manager* _manager,
                                learning::Policy* _policy,
                                simulation::Simulator* _sim0,
                                simulation::Simulator* _sim1) {
    using namespace simulation;
    LOG(INFO) << FUNCTION_NAME();
    bool flagInnerLoopOnSim0 = utils::Option::read("simulation.eval.innerLoopOnSim0").toBool();
    if (flagInnerLoopOnSim0) {
        LOG(INFO) << "Testing the inner loop opimization on Simulation0";
        imp = new LearningGPSimSearchImp;
        imp->manager = _manager;
        imp->s0 = _sim0;
        imp->s1 = NULL;
        imp->policy = _policy;
        LOG(INFO) << "Implementation structure is initialized OK";
        boost::thread t(&worker_innerloopsim0, imp);
        LOG(INFO) << "Testing the inner loop opimization on Simulation0 OK";
        return;
    }

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
    imp->manager = _manager;
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


