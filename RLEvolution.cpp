#include "RLEvolution.h"

#include <string>
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

    int numParams() const {
        return nn->numberOfParameters();
    }

    void setParams(const Eigen::VectorXd& params) {
        shark::RealVector parameters(params.size());
        for (int i = 0; i < params.size(); i++) {
            parameters(i) = params(i);
        }
        nn->setParameterVector(parameters);
    }

    Eigen::VectorXd params() const {
        shark::RealVector parameters = nn->parameterVector();
        Eigen::VectorXd params(parameters.size());
        for (int i = 0; i < params.size(); i++) {
            params(i) = parameters(i);
        }
        return params;
    }

    void create() {
        this->nn = new shark::FFNet<shark::LogisticNeuron,shark::LogisticNeuron>();
        LOG(INFO) << "Neural Network";
        unsigned int numInput  = 6;
        unsigned int numHidden = 4;
        unsigned int numOutput = 1;

        bool connectConsecutiveLayers = true;
        bool connectInputsOutputs     = false;
        bool connectAll               = false;
        bool useBiasNeuron            = false;

        this->nn->setStructure(numInput, numHidden, numOutput,
                              connectConsecutiveLayers, connectInputsOutputs,
                              connectAll, useBiasNeuron);
        LOG(INFO) << "Network connected: # parameters = " << this->nn->numberOfParameters();

        
        Eigen::VectorXd opt(32);
        opt << -4.63813e+07,6.5714e+06,9.02755e+07,3.42653e+07,1.76467e+07,2.71827e+07,-6.64976e+07,-5.24898e+07,3.49514e+06,7.40673e+07,-2.41122e+07,-1.23729e+07,5.33984e+07,50485.7,-1.22745e+08,-1.04219e+08,1.29488e+08,7.34356e+07,-1.67211e+08,-1.90452e+08,-3.5792e+07,-9.19628e+07,-1.62285e+08,-1.38488e+08,-3.11151e+07,-7.48792e+07,-6.63516e+07,2.13303e+08,1.82669e+07,-1.00482e+08,8.93331e+06,5.98239e+07;
        setParams(opt);
    }

    Eigen::VectorXd control(const Eigen::VectorXd& x) {
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
        nn->eval(input, output);
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
    
};
////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////
// // struct RLEvolutionImp
// struct RLEvolutionImp {
//     sim::Simulation* sim;

//     Eigen::MatrixXd K;
//     Eigen::MatrixXd F;
//     Eigen::MatrixXd C;

//     int numParams() const {
//         return 5;
//     }

//     void setParams(const Eigen::VectorXd& params) {
//         for (int i = 0; i < 4; i++) {
//             for (int j = 0; j < 5; j++) {
//                 if (i == 0 || i == 1) {
//                     F(i, j) = params(j);
//                 } else {
//                     F(i, j) = -params(j);
//                 }

//             }
//         }
//         this->K = F * C;
//     }

//     Eigen::VectorXd params() const {
//         return F.row(0);
//     }

//     void create() {
//         int n = 6;
//         // Parameters
//         double radius = 0.05;
//         double rw     = radius;
//         double lrl1 = 1;
//         double lrl2 = 0.1;
//         double lll1 = 1;
//         double lll2 = 0.1;


//         Eigen::MatrixXd C(5, 2 * n);
//         C << - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0, 0, 0, 0, 0, 0, 0,
//             0, 0, 0, 0, 0, 0, - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0,
//             0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,  
//             0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,    
//             1, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0;  
//         this->C = C;
        
//         Eigen::MatrixXd F = Eigen::MatrixXd::Zero(4, 5);
//         this->F = F;
//         this->K = F * C;

//         Eigen::VectorXd opt(5);
//         opt << -8.25341e+07,-1.08371e+07,4.81433e+07,1.273e+07,4.87546e+07;
//         setParams(opt);
            
//     }

//     Eigen::VectorXd control(const Eigen::VectorXd& x) {
//         // Fetch the state
//         int n = x.size() / 2;
//         // Parameters
//         double maxTorq = 200; 

//         // Equilibrium state
//         Eigen::VectorXd qEq(n);
//         qEq << 0.0, 0.0, 0.0, 0.0, PI/2.0, -PI/2.0;
//         Eigen::VectorXd dqEq(n);
//         dqEq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//         Eigen::VectorXd xEq(n * 2);
//         xEq.head(n) = qEq;
//         xEq.tail(n) = dqEq;
        
//         // Calculate u
//         Eigen::VectorXd u = -K * (x - xEq);
//         for(int i = 0; i < u.size(); i++) {
//             if (fabs(u(i)) > maxTorq) {
//                 if (u(i) > 0) {
//                     u(i) = maxTorq;
//                 } else {
//                     u(i) = -maxTorq;
//                 }
//             }
//         }
//         return u;
//     }
    
// };
// ////////////////////////////////////////////////////////////






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
        return imp->numParams();
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

        imp->setParams(params);

        imp->sim->reset();
        for (int i = 0; i < 5000; i++) {
            imp->sim->step();
        }
        double value = imp->sim->getCost();
        LOG(INFO) << "# " << m_evaluationCounter << " : " << params.transpose()
                  << " -> " << value;
        return value;
    }
private:
    RLEvolutionImp* imp;
};
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class RLEvolution implementation
RLEvolution::RLEvolution() {
    imp = new RLEvolutionImp;
    imp->create();
}

RLEvolution::~RLEvolution() {
    // delete imp->nn;
    delete imp;
}

// For training
void RLEvolution::train(sim::Simulation* sim) {
    shark::Rng::seed( (unsigned int) time (NULL) );
    imp->sim = sim;

    PolicyEvaluation prob(imp);
    shark::CMA cma;
    // cma.init( prob );
    // cma.setSigma(10.0);
    shark::RealVector starting(prob.numberOfVariables());
    // cma.init( prob, starting, 32, 16, 15.0 );
    cma.init( prob, starting, 32, 16, 10000000.0 );


    do {
        cma.step( prob );

        // Report information on the optimizer state and the current solution to the console.

        LOG(INFO) << prob.evaluationCounter() << " " << cma.solution().value << " " << cma.solution().point << " " << cma.sigma();

        // Update the params;
        shark::RealVector parameters = cma.solution().point;
        Eigen::VectorXd params(parameters.size());
        for (int i = 0; i < params.size(); i++) {
            params(i) = parameters(i);
        }
        imp->setParams(params);
        save();
    } while(cma.solution().value > (600.0) );
    // imp->nn->setParameterVector( cma.solution().point );

}

void RLEvolution::save() {
    // std::ofstream fout("balance.nn");
    // boost::archive::polymorphic_text_oarchive oa(fout);
    // imp->nn->write(oa);
    // fout.close();    
    // LOG(INFO) << "save balance.nn OK";
}

void RLEvolution::load(const char* const filename) {
    // std::string str_filename;
    // if (filename == NULL) {
    //     str_filename = "balance.nn";
    // } else {
    //     str_filename = filename;
    // }
    // std::ifstream fin(str_filename.c_str());
    // boost::archive::polymorphic_text_iarchive ia(fin);
    // imp->nn->read(ia);
    // fin.close();
    // LOG(INFO) << "load " << str_filename << " OK";
}

// For using
Eigen::VectorXd RLEvolution::control(const Eigen::VectorXd& x) {
    return imp->control(x);
}


// class RLEvolution ends
////////////////////////////////////////////////////////////



} // namespace learning
} // namespace disneysimple



