#include "simulation.h"

#include "cppcommon.h"
#include "RLEvolution.h"

namespace disneysimple {
namespace sim {

Simulation::Simulation()
    : rl(NULL)
{
    init();
    // step();

    // Eigen::VectorXd state = Eigen::VectorXd::Zero(12);
    // state(0) = 0.01;
    // state(1) = 0.02;
    // Eigen::VectorXd control = Eigen::VectorXd::Zero(6);
    // deriv(state, control);
}

Simulation::~Simulation() {
}

void Simulation::init() {
    int n = 6;
    int m = 2 * n;
    Eigen::VectorXd xEq(m);
    xEq << 0, 0, 0, 0, PI_2, -PI_2
         , 0, 0, 0, 0, 0, 0;

    Eigen::VectorXd xOffset(m);
    double angIni = 2;
    xOffset << 0.0, (angIni * PI / 180), 0, 0, 0, 0
        , 0, 0, 0, 0, 0, 0;
        
    mState = xEq + xOffset;

    mStateHistory.clear();
    mStateHistory.push_back(mState);

    mCost = 0.0;
}

void Simulation::control() {
    if (rl != NULL) {
        mTorque = rl->control(mState);
    } else {
        controlFeedback();
    }
}

void Simulation::controlFeedback() {
    int n = nDimConfig();

    // Parameters
    double radius = 0.05;
    double rw     = radius;
    double lrl1 = 1;
    double lrl2 = 0.1;
    double lll1 = 1;
    double lll2 = 0.1;
    double maxTorq = 200; 


    // Feedback Matrix
    Eigen::MatrixXd C(5, 2 * n);
    C << - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,  
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,    
        1, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0;  


    Eigen::MatrixXd F(4, 5);
    F << -2716691.61124073, -1189377.26019503, 953603.332318603, 10071.8805576070, 768507.689768547,
        -2716691.61123261, -1189377.26019358, 953603.332315551, 10071.8805576529, 768507.689765388,
        2716691.61123813, 1189377.26019453, -953603.332317613, -10071.8805576191, -768507.689767548,
        2716691.61124318, 1189377.26019541, -953603.332319511, -10071.8805575885, -768507.689769501;

    // F *= 0.01;

    Eigen::MatrixXd K = F * C;

    // State 
    Eigen::VectorXd x = mState;
    
    // Equilibrium state
    Eigen::VectorXd qEq(n);
    qEq << 0.0, 0.0, 0.0, 0.0, PI/2.0, -PI/2.0;
    Eigen::VectorXd dqEq(n);
    dqEq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd xEq(n * 2);
    xEq.head(n) = qEq;
    xEq.tail(n) = dqEq;
        
    // Calculate u
    Eigen::VectorXd u = -K * (x - xEq);
    for(int i = 0; i < u.size(); i++) {
        if (fabs(u(i)) > maxTorq) {
            if (u(i) > 0) {
                u(i) = maxTorq;
            } else {
                u(i) = -maxTorq;
            }
        }
    }
    mTorque = u;

}

void Simulation::evaluate() {
    // // The first trial to the rest position
    // int n = nDimConfig();
    // const Eigen::VectorXd& x = mState;
    // const Eigen::VectorXd& u = mTorque;

    // Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n * 2, n * 2);
    // Q(0, 0) = 1.0;
    // Q(1, 1) = 1.0;
    // Q(2, 2) = 1.0;
    // for (int i = n; i < n + 3; i++) Q(i, i) = 0.0;
    // Eigen::MatrixXd R = Eigen::MatrixXd::Zero(u.size(), u.size());
    // R(0, 0) = 0.0;
    // double cost = x.dot(Q * x) + u.dot(R * u);


    // mCost += cost;


    // Parameters
    double offset = 0;
    double radius = 0.05;
    double rw     = radius;

    double lrl1 = 1;
    double lrl2 = 0.1;
    double lll1 = 1;
    double lll2 = 0.1;

    // Fetch the state
    int n = nDimConfig();
    Eigen::VectorXd q = mState.head(n);
    double alphaw  = q(0);
    double alphab  = q(1);
    double al      = 0.0;
    double ar      = 0.0;
    double thetal1 = q(2);
    double thetar1 = q(3);
    double thetal2 = q(4);
    double thetar2 = q(5);


    Eigen::Vector3d top;
    top << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - lrl2*sin(alphab + alphaw + thetar1 + thetar2) - rw*sin(alphab + alphaw) - alphaw*rw - lrl1*sin(alphab + alphaw + thetar1),
        rw + lrl2*cos(alphab + alphaw + thetar1 + thetar2) + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw) + lrl1*cos(alphab + alphaw + thetar1)  ;

    // wheel position
    Eigen::Vector3d wheel;
    wheel << 0,
        offset-alphaw*rw,
        rw;
    // feet cart positions
    Eigen::Vector3d rightCart;
    rightCart << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - rw*sin(alphab + alphaw) - alphaw*rw,
        rw + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw);
    
    Eigen::Vector3d leftCart;
    leftCart << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw),
        rw + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw)  ;          
    Eigen::Vector3d cart = 0.5 * (leftCart + rightCart);

    
    // Ignore the Y offset
    const int Y = 2;
    top(Y) = wheel(Y) = cart(Y) = 0.0;

    double cost = (top - wheel).squaredNorm()
        + (top - cart).squaredNorm()
        + (wheel - cart).squaredNorm();
    mCost += cost;

    const Eigen::VectorXd& u = mTorque;
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(u.size(), u.size());
    R(0, 0) = 1e-6;
    mCost += u.dot(R * u);


    // cout << getTime() << " : " << mState.transpose() << " = " << cost << endl;
}

void Simulation::step() {
    control();
    integrate();
    evaluate();
}

void Simulation::loadNN(const char* const filename) {
    if (this->rl == NULL) {
        this->rl = new disneysimple::learning::RLEvolution();
        LOG(INFO) << "Create a instance of RLEvolution class"; 
    }
    this->rl->load(filename);
}

void Simulation::trainNN() {
    if (this->rl == NULL) {
        LOG(WARNING) << FUNCTION_NAME() << " : do not have rl class instance";
        return;
    }
    this->rl = new disneysimple::learning::RLEvolution();
    rl->train(this);
}


/*
// This function's implementation is moved to simulation_deriv.cpp
Eigen::VectorXd
Simulation::deriv(const Eigen::VectorXd& x, const Eigen::VectorXd& control)
*/

/*
  the fourth order Runge-Kutta method
*/
void Simulation::integrate() {
    double h = 0.001;
    double h_2 = 0.5 * h;

    Eigen::VectorXd x = mState;
    Eigen::VectorXd u = mTorque;

    // LOG(INFO) << "x = " << x.transpose();
    // LOG(INFO) << "u = " << u.transpose();

    Eigen::VectorXd k1 = deriv(x, u);
    Eigen::VectorXd k2 = deriv(x + h_2 * k1, u);
    Eigen::VectorXd k3 = deriv(x + h_2 * k2, u);
    Eigen::VectorXd k4 = deriv(x + h * k3, u);

    Eigen::VectorXd dx = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
    // LOG(INFO) << "dx = " << dx.transpose();

    mState = x + h * dx;
    for (int i = 0; i < mState.size(); i++) {
        double x = mState(i);
        if (x < -2.0 * PI) x = -2.0 * PI;
        if (x >  2.0 * PI) x =  2.0 * PI;
        mState(i) = x;
    }
    mStateHistory.push_back(mState);
}


} // namespace sim
} // namespace disneysimple



