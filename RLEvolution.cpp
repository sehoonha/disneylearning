#include "RLEvolution.h"

#include <fstream>
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "cppcommon.h"
#include "simulation.h"

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

namespace disneysimple {
namespace learning {

////////////////////////////////////////////////////////////
// struct RLEvolutionImp
struct RLEvolutionImp {
    shark::FFNet<shark::LogisticNeuron,shark::LogisticNeuron>* nn;
    sim::Simulation* sim;

    void setNNParams(const Eigen::VectorXd& params) {
        shark::RealVector parameters(params.size());
        for (int i = 0; i < params.size(); i++) {
            parameters(i) = params(i);
        }
        nn->setParameterVector(parameters);
    }

    Eigen::VectorXd nnParams() const {
        shark::RealVector parameters = nn->parameterVector();
        Eigen::VectorXd params(parameters.size());
        for (int i = 0; i < params.size(); i++) {
            params(i) = parameters(i);
        }
        return params;
    }

};

////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// Policy Evaluation Function
#define MIN_PARAM -1.0
#define MAX_PARAM 1.0
struct PolicyEvaluation : public shark::SingleObjectiveFunction {

    PolicyEvaluation(RLEvolutionImp* _imp)
        : imp(_imp)
        {
            m_features |= CAN_PROPOSE_STARTING_POINT;
            // m_features |= IS_CONSTRAINED_FEATURE;
            // m_features |= CAN_PROVIDE_CLOSEST_FEASIBLE;
        }
    std::string name() const
        { return "PolicyEvaluation"; }

    std::size_t numberOfVariables()const{
        return imp->nn->numberOfParameters();
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

        imp->setNNParams(params);

        imp->sim->reset();
        for (int i = 0; i < 2000; i++) {
            imp->sim->step();
        }
        double value = imp->sim->getCost();
        LOG(INFO) << "# " << m_evaluationCounter << " : " << value;
        return value;
    }
private:
    RLEvolutionImp* imp;
};
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class RLEvolution implementation
RLEvolution::RLEvolution() {
    // //create network and initialize weights random uniform
    // unsigned numInput=2;
    // unsigned numHidden=12;
    // unsigned numOutput=1;
    // shark::FFNet<shark::LogisticNeuron,shark::LogisticNeuron> network;
    // network.setStructure(numInput,numHidden,numOutput, true, false, false, false);

    // cout << network.numberOfParameters() << endl;        
    // shark::RealVector params(network.numberOfParameters());
    // network.setParameterVector(params);
    // cout << "parameter is updated: " << params << endl;
    // shark::RealVector input(2);
    // input(0) = 0;
    // input(1) = 1;
    // shark::RealVector output;
    // network.eval(input, output);
    // cout << "predict: " << input << " -> " << output << endl;


    imp = new RLEvolutionImp;
    imp->nn = new shark::FFNet<shark::LogisticNeuron,shark::LogisticNeuron>();
    LOG(INFO) << "Neural Network";
    unsigned int numInput  = 6;
    unsigned int numHidden = 4;
    unsigned int numOutput = 1;

    bool connectConsecutiveLayers = true;
    bool connectInputsOutputs     = false;
    bool connectAll               = false;
    bool useBiasNeuron            = false;

    // imp->nn->setStructure(numInput, numHidden, numOutput);
    imp->nn->setStructure(numInput, numHidden, numOutput,
                          connectConsecutiveLayers, connectInputsOutputs,
                          connectAll, useBiasNeuron);
    LOG(INFO) << "Network connected: # parameters = " << imp->nn->numberOfParameters();


    shark::RealVector input(numInput);
    input(0) = 3.0;
    input(1) = 3.0;

    shark::RealVector output(numOutput);
    imp->nn->eval(input, output);
    LOG(INFO) << "Network testing: " << input << " -> " << output;


    // CMA cma;
}

RLEvolution::~RLEvolution() {
    delete imp->nn;
    delete imp;
}

// For training
void RLEvolution::train(sim::Simulation* sim) {
    Eigen::VectorXd opt(32);
    // opt << 12.4229,-16.6791,-28.7858,-15.9828,-40.7481,-23.233,10.7884,-17.428,47.411,6.47651,55.07,8.63919,39.1658,15.0118,-60.2714,-5.32964,17.7993,-29.3988,28.0938,40.8798,-2.74794,-13.7966,-41.9069,47.1869,15.139,-33.0323,31.2539,-17.2525,24.8337,8.64727,0.620035,23.617;
    opt << -19.1507,-4.53068,-41.3912,-38.7542,-46.5239,47.4353,16.212,17.8151,-18.6195,7.42489,-61.2971,28.2913,-8.12158,26.8932,0.089589,3.57483,-52.3572,-4.38877,1.14532,-54.8062,-3.37869,-26.4246,-34.4195,11.4461,-36.5377,23.1277,50.1321,-28.0648,-38.6845,14.1859,16.7063,21.3714; 
    imp->setNNParams(opt);
    save();

    // srand( (unsigned int) time (NULL) );
    // imp->sim = sim;

    // PolicyEvaluation prob(imp);
    // shark::CMA cma;
    // // cma.init( prob );
    // // cma.setSigma(1.0);
    // shark::RealVector starting(prob.numberOfVariables());
        
    // cma.init( prob, starting, 32, 16, 10.0 );


    // do {
    //     cma.step( prob );

    //     // Report information on the optimizer state and the current solution to the console.
    //     LOG(INFO) << prob.evaluationCounter() << " " << cma.solution().value << " " << cma.solution().point << " " << cma.sigma();
    // } while(cma.solution().value > 50.0 );
    // imp->nn->setParameterVector( cma.solution().point );

}

void RLEvolution::save() {
    std::ofstream fout("balance.nn");
    boost::archive::polymorphic_text_oarchive oa(fout);
    imp->nn->write(oa);
    fout.close();    
    LOG(INFO) << "save to balance.nn OK";
}

void RLEvolution::load() {
    std::ifstream fin("balance.nn");
    boost::archive::polymorphic_text_iarchive ia(fin);
    imp->nn->read(ia);
    fin.close();
}

// For using
Eigen::VectorXd RLEvolution::control(const Eigen::VectorXd& x) {
    // Fetch the state
    int n = x.size() / 2;
    Eigen::VectorXd q = x.head(n);
    Eigen::VectorXd dq = x.tail(n);


    shark::RealVector input(6);
    input(0) = q(0);
    input(1) = q(1);
    input(2) = q(2);
    input(3) = dq(0);
    input(4) = dq(1);
    input(5) = dq(2);

    shark::RealVector output;
    // cout << "input... " << input << endl;
    imp->nn->eval(input, output);
    // } catch (shark::Exception& e) {
    //     LOG(FATAL) << "exception!! " << e.what() << " " << e.file() << " " << e.line();

    // }
    // LOG_EVERY_N(INFO, 20) << "predict: " << input << " -> " << output;

    double maxTorq = 200; 
    double o = 2.0 * (output(0) - 0.5) * maxTorq;
    Eigen::VectorXd u(4);
    u << o, o, -o, -o;
    return u;
}


// class RLEvolution ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disneysimple



