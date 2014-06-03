#include "simulation.h"

#include "cppcommon.h"

namespace disneysimple {
namespace sim {
#define pow2(x) ((x) * (x))

Eigen::VectorXd
Simulation::deriv(const Eigen::VectorXd& x, const Eigen::VectorXd& control) {
    int n = nDimConfig();
    Eigen::VectorXd q = x.head(n);
    Eigen::VectorXd dq = x.tail(n);

    // System Parameters
    double pi      = 3.14159265359;
    double g       = 9.81; 
    double radius  = 0.05;
    double maxTorq = 200; 

    double rhow = 200; 
    double rw   = radius;
    double mw   = pi * (rw * rw) * rhow; 
    double Iw   = 0.5 * pi * (rw * rw * rw * rw) * rhow;
    double mb   = 2; 
    double Ib   = 0.1067; 
    double lb   = 0.8; 

    double mrl1 = 15;
    double Irl1 = 1;
    double lrl1 = 1;
    double mrl2 = 15;
    double Irl2 = 2;
    double lrl2 = 0.1;
    double mll1 = 15;
    double Ill1 = 1;
    double lll1 = 1;
    double mll2 = 15;
    double Ill2 = 2;
    double lll2 = 0.1;
    
    // Fetch the state
    double alphaw   = q(0);
    double alphab   = q(1);
    double thetal1  = q(2);
    double thetar1  = q(3);
    double thetal2  = q(4);
    double thetar2  = q(5);

    double dalphaw  = dq(0);
    double dalphab  = dq(1);
    double dthetal1 = dq(2);
    double dthetar1 = dq(3);
    double dthetal2 = dq(4);
    double dthetar2 = dq(5);

    // Calculate the equations of the motions
    //// The mass matrix
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);
    // The first row
    M <<
        Ib + Ill1 + Ill2 + Irl1 + Irl2 + Iw + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll1 + 2*pow2(lll2)*mll2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl1 + 2*pow2(lrl2)*mrl2 + 2*mb*pow2(rw) + 2*mll1*pow2(rw) + 2*mll2*pow2(rw) + 2*mrl1*pow2(rw) + 2*mrl2*pow2(rw) + mw*pow2(rw) + 2*mb*pow2(rw)*cos(alphab + alphaw) + 2*mll1*pow2(rw)*cos(alphab + alphaw) + 2*mll2*pow2(rw)*cos(alphab + alphaw) + 2*mrl1*pow2(rw)*cos(alphab + alphaw) + 2*mrl2*pow2(rw)*cos(alphab + alphaw) - 2*pow2(lll2)*mll2*sin(thetal1 + thetal2) + 2*pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + pow2(alphab)*mb*pow2(rw) + pow2(alphab)*mll1*pow2(rw) + pow2(alphab)*mll2*pow2(rw) + pow2(alphab)*mrl1*pow2(rw) + pow2(alphab)*mrl2*pow2(rw) + 2*alphab*lll2*mll1*rw + 2*alphab*lll2*mll2*rw - 2*alphab*lrl2*mrl1*rw - 2*alphab*lrl2*mrl2*rw + 2*lll2*mll2*rw*cos(alphab + alphaw + thetal1 + thetal2) + 2*lrl2*mrl2*rw*cos(alphab + alphaw + thetar1 + thetar2) + 2*lll2*mll1*rw*sin(alphab + alphaw) + 2*lll2*mll2*rw*sin(alphab + alphaw) - 2*lrl2*mrl1*rw*sin(alphab + alphaw) - 2*lrl2*mrl2*rw*sin(alphab + alphaw) + 2*lll2*mll2*rw*cos(thetal1 + thetal2) + 2*lrl2*mrl2*rw*cos(thetar1 + thetar2) + 2*lll1*lll2*mll2*cos(thetal2) + 2*lrl1*lrl2*mrl2*cos(thetar2) + lll1*mll1*rw*cos(thetal1) + 2*lll1*mll2*rw*cos(thetal1) + lrl1*mrl1*rw*cos(thetar1) + 2*lrl1*mrl2*rw*cos(thetar1) - lll1*lll2*mll1*sin(thetal1) - 2*lll1*lll2*mll2*sin(thetal1) + lrl1*lrl2*mrl1*sin(thetar1) + 2*lrl1*lrl2*mrl2*sin(thetar1) + 2*alphab*mb*pow2(rw)*sin(alphab + alphaw) + 2*alphab*mll1*pow2(rw)*sin(alphab + alphaw) + 2*alphab*mll2*pow2(rw)*sin(alphab + alphaw) + 2*alphab*mrl1*pow2(rw)*sin(alphab + alphaw) + 2*alphab*mrl2*pow2(rw)*sin(alphab + alphaw) + lll1*mll1*rw*cos(alphab + alphaw + thetal1) + 2*lll1*mll2*rw*cos(alphab + alphaw + thetal1) + lrl1*mrl1*rw*cos(alphab + alphaw + thetar1) + 2*lrl1*mrl2*rw*cos(alphab + alphaw + thetar1) - 2*alphab*lll2*mll2*rw*sin(thetal1 + thetal2) - 2*alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - alphab*lll1*mll1*rw*sin(thetal1) - 2*alphab*lll1*mll2*rw*sin(thetal1) - alphab*lrl1*mrl1*rw*sin(thetar1) - 2*alphab*lrl1*mrl2*rw*sin(thetar1),
        Ib + Ill1 + Ill2 + Irl1 + Irl2 + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll1 + 2*pow2(lll2)*mll2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl1 + 2*pow2(lrl2)*mrl2 - 2*pow2(lll2)*mll2*sin(thetal1 + thetal2) + 2*pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + pow2(alphab)*mb*pow2(rw) + pow2(alphab)*mll1*pow2(rw) + pow2(alphab)*mll2*pow2(rw) + pow2(alphab)*mrl1*pow2(rw) + pow2(alphab)*mrl2*pow2(rw) + 2*alphab*lll2*mll1*rw + 2*alphab*lll2*mll2*rw - 2*alphab*lrl2*mrl1*rw - 2*alphab*lrl2*mrl2*rw + lll2*mll2*rw*cos(alphab + alphaw + thetal1 + thetal2) + lrl2*mrl2*rw*cos(alphab + alphaw + thetar1 + thetar2) + lll2*mll1*rw*sin(alphab + alphaw) + lll2*mll2*rw*sin(alphab + alphaw) - lrl2*mrl1*rw*sin(alphab + alphaw) - lrl2*mrl2*rw*sin(alphab + alphaw) + lll2*mll2*rw*cos(thetal1 + thetal2) + lrl2*mrl2*rw*cos(thetar1 + thetar2) + 2*lll1*lll2*mll2*cos(thetal2) + 2*lrl1*lrl2*mrl2*cos(thetar2) + (lll1*mll1*rw*cos(thetal1))/2 + lll1*mll2*rw*cos(thetal1) + (lrl1*mrl1*rw*cos(thetar1))/2 + lrl1*mrl2*rw*cos(thetar1) - lll1*lll2*mll1*sin(thetal1) - 2*lll1*lll2*mll2*sin(thetal1) + lrl1*lrl2*mrl1*sin(thetar1) + 2*lrl1*lrl2*mrl2*sin(thetar1) + alphab*mb*pow2(rw)*sin(alphab + alphaw) + alphab*mll1*pow2(rw)*sin(alphab + alphaw) + alphab*mll2*pow2(rw)*sin(alphab + alphaw) + alphab*mrl1*pow2(rw)*sin(alphab + alphaw) + alphab*mrl2*pow2(rw)*sin(alphab + alphaw) + (lll1*mll1*rw*cos(alphab + alphaw + thetal1))/2 + lll1*mll2*rw*cos(alphab + alphaw + thetal1) + (lrl1*mrl1*rw*cos(alphab + alphaw + thetar1))/2 + lrl1*mrl2*rw*cos(alphab + alphaw + thetar1) - 2*alphab*lll2*mll2*rw*sin(thetal1 + thetal2) - 2*alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - alphab*lll1*mll1*rw*sin(thetal1) - 2*alphab*lll1*mll2*rw*sin(thetal1) - alphab*lrl1*mrl1*rw*sin(thetar1) - 2*alphab*lrl1*mrl2*rw*sin(thetar1),
        Ill1 + Ill2 + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll2 - pow2(lll2)*mll2*sin(thetal1 + thetal2) + lll2*mll2*rw*cos(alphab + alphaw + thetal1 + thetal2) + lll2*mll2*rw*cos(thetal1 + thetal2) + 2*lll1*lll2*mll2*cos(thetal2) + (lll1*mll1*rw*cos(thetal1))/2 + lll1*mll2*rw*cos(thetal1) - (lll1*lll2*mll1*sin(thetal1))/2 - lll1*lll2*mll2*sin(thetal1) + (lll1*mll1*rw*cos(alphab + alphaw + thetal1))/2 + lll1*mll2*rw*cos(alphab + alphaw + thetal1) - alphab*lll2*mll2*rw*sin(thetal1 + thetal2) - (alphab*lll1*mll1*rw*sin(thetal1))/2 - alphab*lll1*mll2*rw*sin(thetal1),
        Irl1 + Irl2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl2 + pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + lrl2*mrl2*rw*cos(alphab + alphaw + thetar1 + thetar2) + lrl2*mrl2*rw*cos(thetar1 + thetar2) + 2*lrl1*lrl2*mrl2*cos(thetar2) + (lrl1*mrl1*rw*cos(thetar1))/2 + lrl1*mrl2*rw*cos(thetar1) + (lrl1*lrl2*mrl1*sin(thetar1))/2 + lrl1*lrl2*mrl2*sin(thetar1) + (lrl1*mrl1*rw*cos(alphab + alphaw + thetar1))/2 + lrl1*mrl2*rw*cos(alphab + alphaw + thetar1) - alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (alphab*lrl1*mrl1*rw*sin(thetar1))/2 - alphab*lrl1*mrl2*rw*sin(thetar1),
        Ill2 + pow2(lll2)*mll2 - pow2(lll2)*mll2*sin(thetal1 + thetal2) + lll2*mll2*rw*cos(alphab + alphaw + thetal1 + thetal2) + lll2*mll2*rw*cos(thetal1 + thetal2) + lll1*lll2*mll2*cos(thetal2) - alphab*lll2*mll2*rw*sin(thetal1 + thetal2),
        Irl2 + pow2(lrl2)*mrl2 + pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + lrl2*mrl2*rw*cos(alphab + alphaw + thetar1 + thetar2) + lrl2*mrl2*rw*cos(thetar1 + thetar2) + lrl1*lrl2*mrl2*cos(thetar2) - alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2),
        // The second row
        Ib + Ill1 + Ill2 + Irl1 + Irl2 + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll1 + 2*pow2(lll2)*mll2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl1 + 2*pow2(lrl2)*mrl2 - 2*pow2(lll2)*mll2*sin(thetal1 + thetal2) + 2*pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + pow2(alphab)*mb*pow2(rw) + pow2(alphab)*mll1*pow2(rw) + pow2(alphab)*mll2*pow2(rw) + pow2(alphab)*mrl1*pow2(rw) + pow2(alphab)*mrl2*pow2(rw) + 2*alphab*lll2*mll1*rw + 2*alphab*lll2*mll2*rw - 2*alphab*lrl2*mrl1*rw - 2*alphab*lrl2*mrl2*rw + lll2*mll2*rw*cos(alphab + alphaw + thetal1 + thetal2) + lrl2*mrl2*rw*cos(alphab + alphaw + thetar1 + thetar2) + lll2*mll1*rw*sin(alphab + alphaw) + lll2*mll2*rw*sin(alphab + alphaw) - lrl2*mrl1*rw*sin(alphab + alphaw) - lrl2*mrl2*rw*sin(alphab + alphaw) + lll2*mll2*rw*cos(thetal1 + thetal2) + lrl2*mrl2*rw*cos(thetar1 + thetar2) + 2*lll1*lll2*mll2*cos(thetal2) + 2*lrl1*lrl2*mrl2*cos(thetar2) + (lll1*mll1*rw*cos(thetal1))/2 + lll1*mll2*rw*cos(thetal1) + (lrl1*mrl1*rw*cos(thetar1))/2 + lrl1*mrl2*rw*cos(thetar1) - lll1*lll2*mll1*sin(thetal1) - 2*lll1*lll2*mll2*sin(thetal1) + lrl1*lrl2*mrl1*sin(thetar1) + 2*lrl1*lrl2*mrl2*sin(thetar1) + alphab*mb*pow2(rw)*sin(alphab + alphaw) + alphab*mll1*pow2(rw)*sin(alphab + alphaw) + alphab*mll2*pow2(rw)*sin(alphab + alphaw) + alphab*mrl1*pow2(rw)*sin(alphab + alphaw) + alphab*mrl2*pow2(rw)*sin(alphab + alphaw) + (lll1*mll1*rw*cos(alphab + alphaw + thetal1))/2 + lll1*mll2*rw*cos(alphab + alphaw + thetal1) + (lrl1*mrl1*rw*cos(alphab + alphaw + thetar1))/2 + lrl1*mrl2*rw*cos(alphab + alphaw + thetar1) - 2*alphab*lll2*mll2*rw*sin(thetal1 + thetal2) - 2*alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - alphab*lll1*mll1*rw*sin(thetal1) - 2*alphab*lll1*mll2*rw*sin(thetal1) - alphab*lrl1*mrl1*rw*sin(thetar1) - 2*alphab*lrl1*mrl2*rw*sin(thetar1),
        Ib + Ill1 + Ill2 + Irl1 + Irl2 + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll1 + 2*pow2(lll2)*mll2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl1 + 2*pow2(lrl2)*mrl2 - 2*pow2(lll2)*mll2*sin(thetal1 + thetal2) + 2*pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + pow2(alphab)*mb*pow2(rw) + pow2(alphab)*mll1*pow2(rw) + pow2(alphab)*mll2*pow2(rw) + pow2(alphab)*mrl1*pow2(rw) + pow2(alphab)*mrl2*pow2(rw) + 2*alphab*lll2*mll1*rw + 2*alphab*lll2*mll2*rw - 2*alphab*lrl2*mrl1*rw - 2*alphab*lrl2*mrl2*rw + 2*lll1*lll2*mll2*cos(thetal2) + 2*lrl1*lrl2*mrl2*cos(thetar2) - lll1*lll2*mll1*sin(thetal1) - 2*lll1*lll2*mll2*sin(thetal1) + lrl1*lrl2*mrl1*sin(thetar1) + 2*lrl1*lrl2*mrl2*sin(thetar1) - 2*alphab*lll2*mll2*rw*sin(thetal1 + thetal2) - 2*alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - alphab*lll1*mll1*rw*sin(thetal1) - 2*alphab*lll1*mll2*rw*sin(thetal1) - alphab*lrl1*mrl1*rw*sin(thetar1) - 2*alphab*lrl1*mrl2*rw*sin(thetar1),
        Ill1 + Ill2 + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll2 - pow2(lll2)*mll2*sin(thetal1 + thetal2) + 2*lll1*lll2*mll2*cos(thetal2) - (lll1*lll2*mll1*sin(thetal1))/2 - lll1*lll2*mll2*sin(thetal1) - alphab*lll2*mll2*rw*sin(thetal1 + thetal2) - (alphab*lll1*mll1*rw*sin(thetal1))/2 - alphab*lll1*mll2*rw*sin(thetal1),
        Irl1 + Irl2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl2 + pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + 2*lrl1*lrl2*mrl2*cos(thetar2) + (lrl1*lrl2*mrl1*sin(thetar1))/2 + lrl1*lrl2*mrl2*sin(thetar1) - alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (alphab*lrl1*mrl1*rw*sin(thetar1))/2 - alphab*lrl1*mrl2*rw*sin(thetar1),
        Ill2 + pow2(lll2)*mll2 - pow2(lll2)*mll2*sin(thetal1 + thetal2) + lll1*lll2*mll2*cos(thetal2) - alphab*lll2*mll2*rw*sin(thetal1 + thetal2),
        Irl2 + pow2(lrl2)*mrl2 + pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + lrl1*lrl2*mrl2*cos(thetar2) - alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2),
        // The third row
        Ill1 + Ill2 + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll2 - pow2(lll2)*mll2*sin(thetal1 + thetal2) + lll2*mll2*rw*cos(alphab + alphaw + thetal1 + thetal2) + lll2*mll2*rw*cos(thetal1 + thetal2) + 2*lll1*lll2*mll2*cos(thetal2) + (lll1*mll1*rw*cos(thetal1))/2 + lll1*mll2*rw*cos(thetal1) - (lll1*lll2*mll1*sin(thetal1))/2 - lll1*lll2*mll2*sin(thetal1) + (lll1*mll1*rw*cos(alphab + alphaw + thetal1))/2 + lll1*mll2*rw*cos(alphab + alphaw + thetal1) - alphab*lll2*mll2*rw*sin(thetal1 + thetal2) - (alphab*lll1*mll1*rw*sin(thetal1))/2 - alphab*lll1*mll2*rw*sin(thetal1),
        Ill1 + Ill2 + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll2 - pow2(lll2)*mll2*sin(thetal1 + thetal2) + 2*lll1*lll2*mll2*cos(thetal2) - (lll1*lll2*mll1*sin(thetal1))/2 - lll1*lll2*mll2*sin(thetal1) - alphab*lll2*mll2*rw*sin(thetal1 + thetal2) - (alphab*lll1*mll1*rw*sin(thetal1))/2 - alphab*lll1*mll2*rw*sin(thetal1),
        Ill1 + Ill2 + (pow2(lll1)*mll1)/4 + pow2(lll1)*mll2 + pow2(lll2)*mll2 + 2*lll1*lll2*mll2*cos(thetal2),
        0,
        mll2*pow2(lll2) + lll1*mll2*cos(thetal2)*lll2 + Ill2,
        0,
        // The fourth row
        Irl1 + Irl2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl2 + pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + lrl2*mrl2*rw*cos(alphab + alphaw + thetar1 + thetar2) + lrl2*mrl2*rw*cos(thetar1 + thetar2) + 2*lrl1*lrl2*mrl2*cos(thetar2) + (lrl1*mrl1*rw*cos(thetar1))/2 + lrl1*mrl2*rw*cos(thetar1) + (lrl1*lrl2*mrl1*sin(thetar1))/2 + lrl1*lrl2*mrl2*sin(thetar1) + (lrl1*mrl1*rw*cos(alphab + alphaw + thetar1))/2 + lrl1*mrl2*rw*cos(alphab + alphaw + thetar1) - alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (alphab*lrl1*mrl1*rw*sin(thetar1))/2 - alphab*lrl1*mrl2*rw*sin(thetar1),
        Irl1 + Irl2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl2 + pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + 2*lrl1*lrl2*mrl2*cos(thetar2) + (lrl1*lrl2*mrl1*sin(thetar1))/2 + lrl1*lrl2*mrl2*sin(thetar1) - alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (alphab*lrl1*mrl1*rw*sin(thetar1))/2 - alphab*lrl1*mrl2*rw*sin(thetar1),
        0,
        Irl1 + Irl2 + (pow2(lrl1)*mrl1)/4 + pow2(lrl1)*mrl2 + pow2(lrl2)*mrl2 + 2*lrl1*lrl2*mrl2*cos(thetar2),
        0,
        mrl2*pow2(lrl2) + lrl1*mrl2*cos(thetar2)*lrl2 + Irl2,
        // The fifth row
        Ill2 + pow2(lll2)*mll2 - pow2(lll2)*mll2*sin(thetal1 + thetal2) + lll2*mll2*rw*cos(alphab + alphaw + thetal1 + thetal2) + lll2*mll2*rw*cos(thetal1 + thetal2) + lll1*lll2*mll2*cos(thetal2) - alphab*lll2*mll2*rw*sin(thetal1 + thetal2),
        Ill2 + pow2(lll2)*mll2 - pow2(lll2)*mll2*sin(thetal1 + thetal2) + lll1*lll2*mll2*cos(thetal2) - alphab*lll2*mll2*rw*sin(thetal1 + thetal2),
        mll2*pow2(lll2) + lll1*mll2*cos(thetal2)*lll2 + Ill2,
        0,
        mll2*pow2(lll2) + Ill2,
        0,
        // The sixth row
        Irl2 + pow2(lrl2)*mrl2 + pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + lrl2*mrl2*rw*cos(alphab + alphaw + thetar1 + thetar2) + lrl2*mrl2*rw*cos(thetar1 + thetar2) + lrl1*lrl2*mrl2*cos(thetar2) - alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2),
        Irl2 + pow2(lrl2)*mrl2 + pow2(lrl2)*mrl2*sin(thetar1 + thetar2) + lrl1*lrl2*mrl2*cos(thetar2) - alphab*lrl2*mrl2*rw*sin(thetar1 + thetar2),
        0,
        mrl2*pow2(lrl2) + lrl1*mrl2*cos(thetar2)*lrl2 + Irl2,
        0,
        mrl2*pow2(lrl2) + Irl2;

    // Coriolis and Centrifugal Matrix    
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, n);
    C <<
        alphab*lll2*mll1*rw + dalphab*lll2*mll2*rw - dalphab*lrl2*mrl1*rw - dalphab*lrl2*mrl2*rw + alphab*dalphab*mb*pow2(rw) + alphab*dalphab*mll1*pow2(rw) + alphab*dalphab*mll2*pow2(rw) + alphab*dalphab*mrl1*pow2(rw) + alphab*dalphab*mrl2*pow2(rw) - dalphaw*mb*pow2(rw)*sin(alphab + alphaw) - dalphaw*mll1*pow2(rw)*sin(alphab + alphaw) - dalphaw*mll2*pow2(rw)*sin(alphab + alphaw) - dalphaw*mrl1*pow2(rw)*sin(alphab + alphaw) - dalphaw*mrl2*pow2(rw)*sin(alphab + alphaw) - dthetal1*pow2(lll2)*mll2*cos(thetal1 + thetal2) - dthetal2*pow2(lll2)*mll2*cos(thetal1 + thetal2) + dthetar1*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + dthetar2*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) - (dalphab*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dalphab*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - (dalphaw*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dalphaw*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - (dalphab*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dalphab*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - (dalphaw*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dalphaw*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - (dthetal1*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dthetal1*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - (dthetar1*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dthetar1*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - dalphab*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dalphaw*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dalphab*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dalphaw*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dthetal1*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dthetal2*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dthetar1*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dthetar2*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) + dalphab*lll2*mll1*rw*cos(alphab + alphaw) + dalphab*lll2*mll2*rw*cos(alphab + alphaw) + dalphaw*lll2*mll1*rw*cos(alphab + alphaw) + dalphaw*lll2*mll2*rw*cos(alphab + alphaw) - dalphab*lrl2*mrl1*rw*cos(alphab + alphaw) - dalphab*lrl2*mrl2*rw*cos(alphab + alphaw) - dalphaw*lrl2*mrl1*rw*cos(alphab + alphaw) - dalphaw*lrl2*mrl2*rw*cos(alphab + alphaw) - dalphab*lll2*mll2*rw*sin(thetal1 + thetal2) - dalphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dthetal1*lll2*mll2*rw*sin(thetal1 + thetal2) - dthetal2*lll2*mll2*rw*sin(thetal1 + thetal2) - dthetar1*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dthetar2*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (dthetal1*lll1*lll2*mll1*cos(thetal1))/2 - dthetal1*lll1*lll2*mll2*cos(thetal1) + (dthetar1*lrl1*lrl2*mrl1*cos(thetar1))/2 + dthetar1*lrl1*lrl2*mrl2*cos(thetar1) + alphab*dalphab*mb*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mb*pow2(rw)*cos(alphab + alphaw) + alphab*dalphab*mll1*pow2(rw)*cos(alphab + alphaw) + alphab*dalphab*mll2*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mll1*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mll2*pow2(rw)*cos(alphab + alphaw) + alphab*dalphab*mrl1*pow2(rw)*cos(alphab + alphaw) + alphab*dalphab*mrl2*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mrl1*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mrl2*pow2(rw)*cos(alphab + alphaw) - dthetal2*lll1*lll2*mll2*sin(thetal2) - dthetar2*lrl1*lrl2*mrl2*sin(thetar2) - (dalphab*lll1*mll1*rw*sin(thetal1))/2 - dalphab*lll1*mll2*rw*sin(thetal1) - (dalphab*lrl1*mrl1*rw*sin(thetar1))/2 - dalphab*lrl1*mrl2*rw*sin(thetar1) - (dthetal1*lll1*mll1*rw*sin(thetal1))/2 - dthetal1*lll1*mll2*rw*sin(thetal1) - (dthetar1*lrl1*mrl1*rw*sin(thetar1))/2 - dthetar1*lrl1*mrl2*rw*sin(thetar1) - alphab*dthetal1*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetal2*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetar1*lrl2*mrl2*rw*cos(thetar1 + thetar2) - alphab*dthetar2*lrl2*mrl2*rw*cos(thetar1 + thetar2) - (alphab*dthetal1*lll1*mll1*rw*cos(thetal1))/2 - alphab*dthetal1*lll1*mll2*rw*cos(thetal1) - (alphab*dthetar1*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*dthetar1*lrl1*mrl2*rw*cos(thetar1),
        2*dalphab*lll2*mll1*rw + 2*dalphab*lll2*mll2*rw + dalphaw*lll2*mll1*rw + dalphaw*lll2*mll2*rw - 2*dalphab*lrl2*mrl1*rw - 2*dalphab*lrl2*mrl2*rw - dalphaw*lrl2*mrl1*rw - dalphaw*lrl2*mrl2*rw + 2*alphab*dalphab*mb*pow2(rw) + alphab*dalphaw*mb*pow2(rw) + 2*alphab*dalphab*mll1*pow2(rw) + 2*alphab*dalphab*mll2*pow2(rw) + alphab*dalphaw*mll1*pow2(rw) + alphab*dalphaw*mll2*pow2(rw) + 2*alphab*dalphab*mrl1*pow2(rw) + 2*alphab*dalphab*mrl2*pow2(rw) + alphab*dalphaw*mrl1*pow2(rw) + alphab*dalphaw*mrl2*pow2(rw) + dalphab*mb*pow2(rw)*sin(alphab + alphaw) + dalphab*mll1*pow2(rw)*sin(alphab + alphaw) + dalphab*mll2*pow2(rw)*sin(alphab + alphaw) + dalphab*mrl1*pow2(rw)*sin(alphab + alphaw) + dalphab*mrl2*pow2(rw)*sin(alphab + alphaw) - dthetal1*pow2(lll2)*mll2*cos(thetal1 + thetal2) - dthetal2*pow2(lll2)*mll2*cos(thetal1 + thetal2) + dthetar1*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + dthetar2*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) - (dalphab*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dalphab*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - (dalphaw*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dalphaw*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - (dalphab*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dalphab*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - (dalphaw*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dalphaw*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - (dthetal1*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dthetal1*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - (dthetar1*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dthetar1*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - dalphab*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dalphaw*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dalphab*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dalphaw*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dthetal1*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dthetal2*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dthetar1*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dthetar2*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) + dalphab*lll2*mll1*rw*cos(alphab + alphaw) + dalphab*lll2*mll2*rw*cos(alphab + alphaw) + dalphaw*lll2*mll1*rw*cos(alphab + alphaw) + dalphaw*lll2*mll2*rw*cos(alphab + alphaw) - dalphab*lrl2*mrl1*rw*cos(alphab + alphaw) - dalphab*lrl2*mrl2*rw*cos(alphab + alphaw) - dalphaw*lrl2*mrl1*rw*cos(alphab + alphaw) - dalphaw*lrl2*mrl2*rw*cos(alphab + alphaw) - 2*dalphab*lll2*mll2*rw*sin(thetal1 + thetal2) - dalphaw*lll2*mll2*rw*sin(thetal1 + thetal2) - 2*dalphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dalphaw*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dthetal1*lll2*mll2*rw*sin(thetal1 + thetal2) - dthetal2*lll2*mll2*rw*sin(thetal1 + thetal2) - dthetar1*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dthetar2*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (dthetal1*lll1*lll2*mll1*cos(thetal1))/2 - dthetal1*lll1*lll2*mll2*cos(thetal1) + (dthetar1*lrl1*lrl2*mrl1*cos(thetar1))/2 + dthetar1*lrl1*lrl2*mrl2*cos(thetar1) + alphab*dalphab*mb*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mb*pow2(rw)*cos(alphab + alphaw) + alphab*dalphab*mll1*pow2(rw)*cos(alphab + alphaw) + alphab*dalphab*mll2*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mll1*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mll2*pow2(rw)*cos(alphab + alphaw) + alphab*dalphab*mrl1*pow2(rw)*cos(alphab + alphaw) + alphab*dalphab*mrl2*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mrl1*pow2(rw)*cos(alphab + alphaw) + alphab*dalphaw*mrl2*pow2(rw)*cos(alphab + alphaw) - dthetal2*lll1*lll2*mll2*sin(thetal2) - dthetar2*lrl1*lrl2*mrl2*sin(thetar2) - dalphab*lll1*mll1*rw*sin(thetal1) - 2*dalphab*lll1*mll2*rw*sin(thetal1) - (dalphaw*lll1*mll1*rw*sin(thetal1))/2 - dalphaw*lll1*mll2*rw*sin(thetal1) - dalphab*lrl1*mrl1*rw*sin(thetar1) - 2*dalphab*lrl1*mrl2*rw*sin(thetar1) - (dalphaw*lrl1*mrl1*rw*sin(thetar1))/2 - dalphaw*lrl1*mrl2*rw*sin(thetar1) - (dthetal1*lll1*mll1*rw*sin(thetal1))/2 - dthetal1*lll1*mll2*rw*sin(thetal1) - (dthetar1*lrl1*mrl1*rw*sin(thetar1))/2 - dthetar1*lrl1*mrl2*rw*sin(thetar1) - alphab*dthetal1*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetal2*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetar1*lrl2*mrl2*rw*cos(thetar1 + thetar2) - alphab*dthetar2*lrl2*mrl2*rw*cos(thetar1 + thetar2) - (alphab*dthetal1*lll1*mll1*rw*cos(thetal1))/2 - alphab*dthetal1*lll1*mll2*rw*cos(thetal1) - (alphab*dthetar1*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*dthetar1*lrl1*mrl2*rw*cos(thetar1),
        - dalphab*pow2(lll2)*mll2*cos(thetal1 + thetal2) - dalphaw*pow2(lll2)*mll2*cos(thetal1 + thetal2) - dthetal1*pow2(lll2)*mll2*cos(thetal1 + thetal2) - dthetal2*pow2(lll2)*mll2*cos(thetal1 + thetal2) - (dalphab*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dalphab*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - (dalphaw*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dalphaw*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - (dthetal1*lll1*mll1*rw*sin(alphab + alphaw + thetal1))/2 - dthetal1*lll1*mll2*rw*sin(alphab + alphaw + thetal1) - dalphab*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dalphaw*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dthetal1*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dthetal2*lll2*mll2*rw*sin(alphab + alphaw + thetal1 + thetal2) - dalphab*lll2*mll2*rw*sin(thetal1 + thetal2) - dalphaw*lll2*mll2*rw*sin(thetal1 + thetal2) - dthetal1*lll2*mll2*rw*sin(thetal1 + thetal2) - dthetal2*lll2*mll2*rw*sin(thetal1 + thetal2) - (dalphab*lll1*lll2*mll1*cos(thetal1))/2 - dalphab*lll1*lll2*mll2*cos(thetal1) - (dalphaw*lll1*lll2*mll1*cos(thetal1))/2 - dalphaw*lll1*lll2*mll2*cos(thetal1) - (dthetal1*lll1*lll2*mll1*cos(thetal1))/2 - dthetal1*lll1*lll2*mll2*cos(thetal1) - dthetal2*lll1*lll2*mll2*sin(thetal2) - (dalphab*lll1*mll1*rw*sin(thetal1))/2 - dalphab*lll1*mll2*rw*sin(thetal1) - (dalphaw*lll1*mll1*rw*sin(thetal1))/2 - dalphaw*lll1*mll2*rw*sin(thetal1) - (dthetal1*lll1*mll1*rw*sin(thetal1))/2 - dthetal1*lll1*mll2*rw*sin(thetal1) - alphab*dalphab*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dalphaw*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetal1*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetal2*lll2*mll2*rw*cos(thetal1 + thetal2) - (alphab*dalphab*lll1*mll1*rw*cos(thetal1))/2 - alphab*dalphab*lll1*mll2*rw*cos(thetal1) - (alphab*dalphaw*lll1*mll1*rw*cos(thetal1))/2 - alphab*dalphaw*lll1*mll2*rw*cos(thetal1) - (alphab*dthetal1*lll1*mll1*rw*cos(thetal1))/2 - alphab*dthetal1*lll1*mll2*rw*cos(thetal1),
        dalphab*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + dalphaw*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + dthetar1*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + dthetar2*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) - (dalphab*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dalphab*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - (dalphaw*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dalphaw*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - (dthetar1*lrl1*mrl1*rw*sin(alphab + alphaw + thetar1))/2 - dthetar1*lrl1*mrl2*rw*sin(alphab + alphaw + thetar1) - dalphab*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dalphaw*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dthetar1*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dthetar2*lrl2*mrl2*rw*sin(alphab + alphaw + thetar1 + thetar2) - dalphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dalphaw*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dthetar1*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dthetar2*lrl2*mrl2*rw*sin(thetar1 + thetar2) + (dalphab*lrl1*lrl2*mrl1*cos(thetar1))/2 + dalphab*lrl1*lrl2*mrl2*cos(thetar1) + (dalphaw*lrl1*lrl2*mrl1*cos(thetar1))/2 + dalphaw*lrl1*lrl2*mrl2*cos(thetar1) + (dthetar1*lrl1*lrl2*mrl1*cos(thetar1))/2 + dthetar1*lrl1*lrl2*mrl2*cos(thetar1) - dthetar2*lrl1*lrl2*mrl2*sin(thetar2) - (dalphab*lrl1*mrl1*rw*sin(thetar1))/2 - dalphab*lrl1*mrl2*rw*sin(thetar1) - (dalphaw*lrl1*mrl1*rw*sin(thetar1))/2 - dalphaw*lrl1*mrl2*rw*sin(thetar1) - (dthetar1*lrl1*mrl1*rw*sin(thetar1))/2 - dthetar1*lrl1*mrl2*rw*sin(thetar1) - alphab*dalphab*lrl2*mrl2*rw*cos(thetar1 + thetar2) - alphab*dalphaw*lrl2*mrl2*rw*cos(thetar1 + thetar2) - alphab*dthetar1*lrl2*mrl2*rw*cos(thetar1 + thetar2) - alphab*dthetar2*lrl2*mrl2*rw*cos(thetar1 + thetar2) - (alphab*dalphab*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*dalphab*lrl1*mrl2*rw*cos(thetar1) - (alphab*dalphaw*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*dalphaw*lrl1*mrl2*rw*cos(thetar1) - (alphab*dthetar1*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*dthetar1*lrl1*mrl2*rw*cos(thetar1),
        -lll2*mll2*(dalphab + dalphaw + dthetal1 + dthetal2)*(rw*sin(alphab + alphaw + thetal1 + thetal2) + lll2*cos(thetal1 + thetal2) + rw*sin(thetal1 + thetal2) + lll1*sin(thetal2) + alphab*rw*cos(thetal1 + thetal2)),
        -lrl2*mrl2*(dalphab + dalphaw + dthetar1 + dthetar2)*(rw*sin(alphab + alphaw + thetar1 + thetar2) - lrl2*cos(thetar1 + thetar2) + rw*sin(thetar1 + thetar2) + lrl1*sin(thetar2) + alphab*rw*cos(thetar1 + thetar2)),
        // The second row
        dalphaw*lrl2*mrl1*rw - dalphaw*lll2*mll2*rw - dalphaw*lll2*mll1*rw + dalphaw*lrl2*mrl2*rw - alphab*dalphaw*mb*pow2(rw) - alphab*dalphaw*mll1*pow2(rw) - alphab*dalphaw*mll2*pow2(rw) - alphab*dalphaw*mrl1*pow2(rw) - alphab*dalphaw*mrl2*pow2(rw) - dthetal1*pow2(lll2)*mll2*cos(thetal1 + thetal2) - dthetal2*pow2(lll2)*mll2*cos(thetal1 + thetal2) + dthetar1*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + dthetar2*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + dalphaw*lll2*mll2*rw*sin(thetal1 + thetal2) + dalphaw*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (dthetal1*lll1*lll2*mll1*cos(thetal1))/2 - dthetal1*lll1*lll2*mll2*cos(thetal1) + (dthetar1*lrl1*lrl2*mrl1*cos(thetar1))/2 + dthetar1*lrl1*lrl2*mrl2*cos(thetar1) - dthetal2*lll1*lll2*mll2*sin(thetal2) - dthetar2*lrl1*lrl2*mrl2*sin(thetar2) + (dalphaw*lll1*mll1*rw*sin(thetal1))/2 + dalphaw*lll1*mll2*rw*sin(thetal1) + (dalphaw*lrl1*mrl1*rw*sin(thetar1))/2 + dalphaw*lrl1*mrl2*rw*sin(thetar1) - alphab*dthetal1*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetal2*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetar1*lrl2*mrl2*rw*cos(thetar1 + thetar2) - alphab*dthetar2*lrl2*mrl2*rw*cos(thetar1 + thetar2) - (alphab*dthetal1*lll1*mll1*rw*cos(thetal1))/2 - alphab*dthetal1*lll1*mll2*rw*cos(thetal1) - (alphab*dthetar1*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*dthetar1*lrl1*mrl2*rw*cos(thetar1),
        dalphab*lll2*mll1*rw + dalphab*lll2*mll2*rw - dalphab*lrl2*mrl1*rw - dalphab*lrl2*mrl2*rw + alphab*dalphab*mb*pow2(rw) + alphab*dalphab*mll1*pow2(rw) + alphab*dalphab*mll2*pow2(rw) + alphab*dalphab*mrl1*pow2(rw) + alphab*dalphab*mrl2*pow2(rw) - dthetal1*pow2(lll2)*mll2*cos(thetal1 + thetal2) - dthetal2*pow2(lll2)*mll2*cos(thetal1 + thetal2) + dthetar1*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + dthetar2*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) - dalphab*lll2*mll2*rw*sin(thetal1 + thetal2) - dalphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (dthetal1*lll1*lll2*mll1*cos(thetal1))/2 - dthetal1*lll1*lll2*mll2*cos(thetal1) + (dthetar1*lrl1*lrl2*mrl1*cos(thetar1))/2 + dthetar1*lrl1*lrl2*mrl2*cos(thetar1) - dthetal2*lll1*lll2*mll2*sin(thetal2) - dthetar2*lrl1*lrl2*mrl2*sin(thetar2) - (dalphab*lll1*mll1*rw*sin(thetal1))/2 - dalphab*lll1*mll2*rw*sin(thetal1) - (dalphab*lrl1*mrl1*rw*sin(thetar1))/2 - dalphab*lrl1*mrl2*rw*sin(thetar1) - alphab*dthetal1*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetal2*lll2*mll2*rw*cos(thetal1 + thetal2) - alphab*dthetar1*lrl2*mrl2*rw*cos(thetar1 + thetar2) - alphab*dthetar2*lrl2*mrl2*rw*cos(thetar1 + thetar2) - (alphab*dthetal1*lll1*mll1*rw*cos(thetal1))/2 - alphab*dthetal1*lll1*mll2*rw*cos(thetal1) - (alphab*dthetar1*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*dthetar1*lrl1*mrl2*rw*cos(thetar1),
        - dthetal2*(pow2(lll2)*mll2*cos(thetal1 + thetal2) + lll1*lll2*mll2*sin(thetal2) + alphab*lll2*mll2*rw*cos(thetal1 + thetal2)) - dalphab*(pow2(lll2)*mll2*cos(thetal1 + thetal2) + (lll1*lll2*mll1*cos(thetal1))/2 + lll1*lll2*mll2*cos(thetal1) + alphab*lll2*mll2*rw*cos(thetal1 + thetal2) + (alphab*lll1*mll1*rw*cos(thetal1))/2 + alphab*lll1*mll2*rw*cos(thetal1)) - dalphaw*(pow2(lll2)*mll2*cos(thetal1 + thetal2) + (lll1*lll2*mll1*cos(thetal1))/2 + lll1*lll2*mll2*cos(thetal1) + alphab*lll2*mll2*rw*cos(thetal1 + thetal2) + (alphab*lll1*mll1*rw*cos(thetal1))/2 + alphab*lll1*mll2*rw*cos(thetal1)) - dthetal1*(pow2(lll2)*mll2*cos(thetal1 + thetal2) + (lll1*lll2*mll1*cos(thetal1))/2 + lll1*lll2*mll2*cos(thetal1) + alphab*lll2*mll2*rw*cos(thetal1 + thetal2) + (alphab*lll1*mll1*rw*cos(thetal1))/2 + alphab*lll1*mll2*rw*cos(thetal1)),
        dalphab*(pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + (lrl1*lrl2*mrl1*cos(thetar1))/2 + lrl1*lrl2*mrl2*cos(thetar1) - alphab*lrl2*mrl2*rw*cos(thetar1 + thetar2) - (alphab*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*lrl1*mrl2*rw*cos(thetar1)) - dthetar2*(lrl1*lrl2*mrl2*sin(thetar2) - pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + alphab*lrl2*mrl2*rw*cos(thetar1 + thetar2)) + dalphaw*(pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + (lrl1*lrl2*mrl1*cos(thetar1))/2 + lrl1*lrl2*mrl2*cos(thetar1) - alphab*lrl2*mrl2*rw*cos(thetar1 + thetar2) - (alphab*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*lrl1*mrl2*rw*cos(thetar1)) + dthetar1*(pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + (lrl1*lrl2*mrl1*cos(thetar1))/2 + lrl1*lrl2*mrl2*cos(thetar1) - alphab*lrl2*mrl2*rw*cos(thetar1 + thetar2) - (alphab*lrl1*mrl1*rw*cos(thetar1))/2 - alphab*lrl1*mrl2*rw*cos(thetar1)),
        -lll2*mll2*(lll2*cos(thetal1 + thetal2) + lll1*sin(thetal2) + alphab*rw*cos(thetal1 + thetal2))*(dalphab + dalphaw + dthetal1 + dthetal2),
        -lrl2*mrl2*(lrl1*sin(thetar2) - lrl2*cos(thetar1 + thetar2) + alphab*rw*cos(thetar1 + thetar2))*(dalphab + dalphaw + dthetar1 + dthetar2),
        // The third row
        dalphab*pow2(lll2)*mll2*cos(thetal1 + thetal2) + dalphaw*pow2(lll2)*mll2*cos(thetal1 + thetal2) + dalphaw*lll2*mll2*rw*sin(thetal1 + thetal2) + (dalphab*lll1*lll2*mll1*cos(thetal1))/2 + dalphab*lll1*lll2*mll2*cos(thetal1) + (dalphaw*lll1*lll2*mll1*cos(thetal1))/2 + dalphaw*lll1*lll2*mll2*cos(thetal1) - dthetal2*lll1*lll2*mll2*sin(thetal2) + (dalphaw*lll1*mll1*rw*sin(thetal1))/2 + dalphaw*lll1*mll2*rw*sin(thetal1) + alphab*dalphab*lll2*mll2*rw*cos(thetal1 + thetal2) + alphab*dalphaw*lll2*mll2*rw*cos(thetal1 + thetal2) + (alphab*dalphab*lll1*mll1*rw*cos(thetal1))/2 + alphab*dalphab*lll1*mll2*rw*cos(thetal1) + (alphab*dalphaw*lll1*mll1*rw*cos(thetal1))/2 + alphab*dalphaw*lll1*mll2*rw*cos(thetal1),
        dalphab*pow2(lll2)*mll2*cos(thetal1 + thetal2) + dalphaw*pow2(lll2)*mll2*cos(thetal1 + thetal2) - dalphab*lll2*mll2*rw*sin(thetal1 + thetal2) + (dalphab*lll1*lll2*mll1*cos(thetal1))/2 + dalphab*lll1*lll2*mll2*cos(thetal1) + (dalphaw*lll1*lll2*mll1*cos(thetal1))/2 + dalphaw*lll1*lll2*mll2*cos(thetal1) - dthetal2*lll1*lll2*mll2*sin(thetal2) - (dalphab*lll1*mll1*rw*sin(thetal1))/2 - dalphab*lll1*mll2*rw*sin(thetal1) + alphab*dalphab*lll2*mll2*rw*cos(thetal1 + thetal2) + alphab*dalphaw*lll2*mll2*rw*cos(thetal1 + thetal2) + (alphab*dalphab*lll1*mll1*rw*cos(thetal1))/2 + alphab*dalphab*lll1*mll2*rw*cos(thetal1) + (alphab*dalphaw*lll1*mll1*rw*cos(thetal1))/2 + alphab*dalphaw*lll1*mll2*rw*cos(thetal1),
        -dthetal2*lll1*lll2*mll2*sin(thetal2),
        0,
        -lll1*lll2*mll2*sin(thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2),
        0,
        // The fourth row
        dalphaw*lrl2*mrl2*rw*sin(thetar1 + thetar2) - dalphaw*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) - dalphab*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) - (dalphab*lrl1*lrl2*mrl1*cos(thetar1))/2 - dalphab*lrl1*lrl2*mrl2*cos(thetar1) - (dalphaw*lrl1*lrl2*mrl1*cos(thetar1))/2 - dalphaw*lrl1*lrl2*mrl2*cos(thetar1) - dthetar2*lrl1*lrl2*mrl2*sin(thetar2) + (dalphaw*lrl1*mrl1*rw*sin(thetar1))/2 + dalphaw*lrl1*mrl2*rw*sin(thetar1) + alphab*dalphab*lrl2*mrl2*rw*cos(thetar1 + thetar2) + alphab*dalphaw*lrl2*mrl2*rw*cos(thetar1 + thetar2) + (alphab*dalphab*lrl1*mrl1*rw*cos(thetar1))/2 + alphab*dalphab*lrl1*mrl2*rw*cos(thetar1) + (alphab*dalphaw*lrl1*mrl1*rw*cos(thetar1))/2 + alphab*dalphaw*lrl1*mrl2*rw*cos(thetar1),
        alphab*dalphab*lrl2*mrl2*rw*cos(thetar1 + thetar2) - dalphaw*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) - dalphab*lrl2*mrl2*rw*sin(thetar1 + thetar2) - (dalphab*lrl1*lrl2*mrl1*cos(thetar1))/2 - dalphab*lrl1*lrl2*mrl2*cos(thetar1) - (dalphaw*lrl1*lrl2*mrl1*cos(thetar1))/2 - dalphaw*lrl1*lrl2*mrl2*cos(thetar1) - dthetar2*lrl1*lrl2*mrl2*sin(thetar2) - (dalphab*lrl1*mrl1*rw*sin(thetar1))/2 - dalphab*lrl1*mrl2*rw*sin(thetar1) - dalphab*pow2(lrl2)*mrl2*cos(thetar1 + thetar2) + alphab*dalphaw*lrl2*mrl2*rw*cos(thetar1 + thetar2) + (alphab*dalphab*lrl1*mrl1*rw*cos(thetar1))/2 + alphab*dalphab*lrl1*mrl2*rw*cos(thetar1) + (alphab*dalphaw*lrl1*mrl1*rw*cos(thetar1))/2 + alphab*dalphaw*lrl1*mrl2*rw*cos(thetar1),
        0,
        -dthetar2*lrl1*lrl2*mrl2*sin(thetar2),
        0,
        -lrl1*lrl2*mrl2*sin(thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2),
        // The fifth row
        lll2*mll2*(dalphab*lll1*sin(thetal2) + dalphaw*lll1*sin(thetal2) + dthetal1*lll1*sin(thetal2) + dalphab*lll2*cos(thetal1 + thetal2) + dalphaw*lll2*cos(thetal1 + thetal2) + dalphaw*rw*sin(thetal1 + thetal2) + alphab*dalphab*rw*cos(thetal1 + thetal2) + alphab*dalphaw*rw*cos(thetal1 + thetal2)),
        lll2*mll2*(dalphab*lll1*sin(thetal2) + dalphaw*lll1*sin(thetal2) + dthetal1*lll1*sin(thetal2) + dalphab*lll2*cos(thetal1 + thetal2) + dalphaw*lll2*cos(thetal1 + thetal2) - dalphab*rw*sin(thetal1 + thetal2) + alphab*dalphab*rw*cos(thetal1 + thetal2) + alphab*dalphaw*rw*cos(thetal1 + thetal2)),
        lll1*lll2*mll2*sin(thetal2)*(dalphab + dalphaw + dthetal1),
        0,
        0,
        0,
        // The sixth row
        lrl2*mrl2*(dalphab*lrl1*sin(thetar2) + dalphaw*lrl1*sin(thetar2) + dthetar1*lrl1*sin(thetar2) - dalphab*lrl2*cos(thetar1 + thetar2) - dalphaw*lrl2*cos(thetar1 + thetar2) + dalphaw*rw*sin(thetar1 + thetar2) + alphab*dalphab*rw*cos(thetar1 + thetar2) + alphab*dalphaw*rw*cos(thetar1 + thetar2)),
        lrl2*mrl2*(dalphab*lrl1*sin(thetar2) + dalphaw*lrl1*sin(thetar2) + dthetar1*lrl1*sin(thetar2) - dalphab*lrl2*cos(thetar1 + thetar2) - dalphaw*lrl2*cos(thetar1 + thetar2) - dalphab*rw*sin(thetar1 + thetar2) + alphab*dalphab*rw*cos(thetar1 + thetar2) + alphab*dalphaw*rw*cos(thetar1 + thetar2)),
        0,
        lrl1*lrl2*mrl2*sin(thetar2)*(dalphab + dalphaw + dthetar1),
        0,
        0;

    // Vector of Gravitational Forces
    Eigen::MatrixXd G(n, 1);
    G <<
        - g*mb*(rw*sin(alphab + alphaw) - alphab*rw*cos(alphab + alphaw)) - g*mll1*(rw*sin(alphab + alphaw) + (lll1*sin(alphab + alphaw + thetal1))/2 - cos(alphab + alphaw)*(lll2 + alphab*rw)) - g*mrl1*(rw*sin(alphab + alphaw) + (lrl1*sin(alphab + alphaw + thetar1))/2 + cos(alphab + alphaw)*(lrl2 - alphab*rw)) - g*mll2*(lll2*sin(alphab + alphaw + thetal1 + thetal2) + rw*sin(alphab + alphaw) + lll1*sin(alphab + alphaw + thetal1) - cos(alphab + alphaw)*(lll2 + alphab*rw)) - g*mrl2*(lrl2*sin(alphab + alphaw + thetar1 + thetar2) + rw*sin(alphab + alphaw) + lrl1*sin(alphab + alphaw + thetar1) + cos(alphab + alphaw)*(lrl2 - alphab*rw)),
        alphab*g*mb*rw*cos(alphab + alphaw) - g*mrl1*((lrl1*sin(alphab + alphaw + thetar1))/2 + cos(alphab + alphaw)*(lrl2 - alphab*rw)) - g*mll2*(lll2*sin(alphab + alphaw + thetal1 + thetal2) + lll1*sin(alphab + alphaw + thetal1) - cos(alphab + alphaw)*(lll2 + alphab*rw)) - g*mrl2*(lrl2*sin(alphab + alphaw + thetar1 + thetar2) + lrl1*sin(alphab + alphaw + thetar1) + cos(alphab + alphaw)*(lrl2 - alphab*rw)) - g*mll1*((lll1*sin(alphab + alphaw + thetal1))/2 - cos(alphab + alphaw)*(lll2 + alphab*rw)),
        - g*mll2*(lll2*sin(alphab + alphaw + thetal1 + thetal2) + lll1*sin(alphab + alphaw + thetal1)) - (g*lll1*mll1*sin(alphab + alphaw + thetal1))/2,
        - g*mrl2*(lrl2*sin(alphab + alphaw + thetar1 + thetar2) + lrl1*sin(alphab + alphaw + thetar1)) - (g*lrl1*mrl1*sin(alphab + alphaw + thetar1))/2,
        -g*lll2*mll2*sin(alphab + alphaw + thetal1 + thetal2),
        -g*lrl2*mrl2*sin(alphab + alphaw + thetar1 + thetar2);
    
    // Constraint Matrix
    Eigen::MatrixXd Psi(3, n); // Three constraints
    Psi <<
        // The first row
        lrl2*cos(alphab + alphaw + thetar1 + thetar2) - lll2*cos(alphab + alphaw + thetal1 + thetal2) - lll2*sin(alphab + alphaw) - lrl2*sin(alphab + alphaw) - lll1*cos(alphab + alphaw + thetal1) + lrl1*cos(alphab + alphaw + thetar1),
        lrl2*cos(alphab + alphaw + thetar1 + thetar2) - lll2*cos(alphab + alphaw + thetal1 + thetal2) - lll2*sin(alphab + alphaw) - lrl2*sin(alphab + alphaw) - lll1*cos(alphab + alphaw + thetal1) + lrl1*cos(alphab + alphaw + thetar1),
        - lll2*cos(alphab + alphaw + thetal1 + thetal2) - lll1*cos(alphab + alphaw + thetal1),
        lrl2*cos(alphab + alphaw + thetar1 + thetar2) + lrl1*cos(alphab + alphaw + thetar1),
        -lll2*cos(alphab + alphaw + thetal1 + thetal2),
        lrl2*cos(alphab + alphaw + thetar1 + thetar2),
        // The second row
        lrl2*sin(alphab + alphaw + thetar1 + thetar2) - lll2*sin(alphab + alphaw + thetal1 + thetal2) + lll2*cos(alphab + alphaw) + lrl2*cos(alphab + alphaw) - lll1*sin(alphab + alphaw + thetal1) + lrl1*sin(alphab + alphaw + thetar1),
        lrl2*sin(alphab + alphaw + thetar1 + thetar2) - lll2*sin(alphab + alphaw + thetal1 + thetal2) + lll2*cos(alphab + alphaw) + lrl2*cos(alphab + alphaw) - lll1*sin(alphab + alphaw + thetal1) + lrl1*sin(alphab + alphaw + thetar1),
        - lll2*sin(alphab + alphaw + thetal1 + thetal2) - lll1*sin(alphab + alphaw + thetal1),
        lrl2*sin(alphab + alphaw + thetar1 + thetar2) + lrl1*sin(alphab + alphaw + thetar1),
        -lll2*sin(alphab + alphaw + thetal1 + thetal2),
        lrl2*sin(alphab + alphaw + thetar1 + thetar2),
        // The third row
        0,
        0,
        1,
        -1,
        1,
        -1;

    Eigen::MatrixXd PsiDot(3, n);
    PsiDot <<
        // The first row
        lll2*sin(alphab + alphaw + thetal1 + thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2) - lrl2*sin(alphab + alphaw + thetar1 + thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2) + lll1*sin(alphab + alphaw + thetal1)*(dalphab + dalphaw + dthetal1) - lrl1*sin(alphab + alphaw + thetar1)*(dalphab + dalphaw + dthetar1) - lll2*cos(alphab + alphaw)*(dalphab + dalphaw) - lrl2*cos(alphab + alphaw)*(dalphab + dalphaw),
        lll2*sin(alphab + alphaw + thetal1 + thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2) - lrl2*sin(alphab + alphaw + thetar1 + thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2) + lll1*sin(alphab + alphaw + thetal1)*(dalphab + dalphaw + dthetal1) - lrl1*sin(alphab + alphaw + thetar1)*(dalphab + dalphaw + dthetar1) - lll2*cos(alphab + alphaw)*(dalphab + dalphaw) - lrl2*cos(alphab + alphaw)*(dalphab + dalphaw),
        lll2*sin(alphab + alphaw + thetal1 + thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2) + lll1*sin(alphab + alphaw + thetal1)*(dalphab + dalphaw + dthetal1),
        - lrl2*sin(alphab + alphaw + thetar1 + thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2) - lrl1*sin(alphab + alphaw + thetar1)*(dalphab + dalphaw + dthetar1),
        lll2*sin(alphab + alphaw + thetal1 + thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2),
        -lrl2*sin(alphab + alphaw + thetar1 + thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2),
        // The second row
        lrl2*cos(alphab + alphaw + thetar1 + thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2) - lll2*cos(alphab + alphaw + thetal1 + thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2) - lll1*cos(alphab + alphaw + thetal1)*(dalphab + dalphaw + dthetal1) + lrl1*cos(alphab + alphaw + thetar1)*(dalphab + dalphaw + dthetar1) - lll2*sin(alphab + alphaw)*(dalphab + dalphaw) - lrl2*sin(alphab + alphaw)*(dalphab + dalphaw),
        lrl2*cos(alphab + alphaw + thetar1 + thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2) - lll2*cos(alphab + alphaw + thetal1 + thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2) - lll1*cos(alphab + alphaw + thetal1)*(dalphab + dalphaw + dthetal1) + lrl1*cos(alphab + alphaw + thetar1)*(dalphab + dalphaw + dthetar1) - lll2*sin(alphab + alphaw)*(dalphab + dalphaw) - lrl2*sin(alphab + alphaw)*(dalphab + dalphaw),
        - lll2*cos(alphab + alphaw + thetal1 + thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2) - lll1*cos(alphab + alphaw + thetal1)*(dalphab + dalphaw + dthetal1),
        lrl2*cos(alphab + alphaw + thetar1 + thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2) + lrl1*cos(alphab + alphaw + thetar1)*(dalphab + dalphaw + dthetar1),
        -lll2*cos(alphab + alphaw + thetal1 + thetal2)*(dalphab + dalphaw + dthetal1 + dthetal2),
        lrl2*cos(alphab + alphaw + thetar1 + thetar2)*(dalphab + dalphaw + dthetar1 + dthetar2),
        // The third row
        0,
        0,
        0,
        0,
        0,
        0;    

    // Stiffness matrix
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(4, n * 2);

    // Actuation Matrix
    Eigen::MatrixXd F(n, 4);
    F << 0, 0, 0, 0,
        0, 0, 0, 0,
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Equilibrium state
    Eigen::VectorXd qEq(n);
    qEq << 0.0, 0.0, 0.0, 0.0, pi/2.0, -pi/2.0;
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

    // Generalized force vector
    Eigen::MatrixXd U = F*u;

    // Disturbance
    double df = 0.0; // Now I assume no disturbance
    Eigen::VectorXd D(n);
    D << 0, df, 0, 0, 0, 0;
    double dv = 0.0;
    Eigen::VectorXd Dv(n);
    Dv << dv * dalphaw, 0, 0, 0, 0, 0;

    // Lagrange Multiplier
    Eigen::MatrixXd invM = M.inverse();
    Eigen::MatrixXd PsiT = Psi.transpose();
    Eigen::VectorXd lagMult =
        - ((Psi * (invM * PsiT)).inverse())
        * (PsiDot*dq + Psi*(invM * (U + D - Dv - G - C*dq)));

    // Derivatives
    Eigen::VectorXd ddq = invM * ( U + D - G - C*dq + PsiT * lagMult);
    Eigen::VectorXd dx(n * 2);
    dx.head(n) = dq;
    dx.tail(n) = ddq;

    // LOG(INFO) << "q = " << q.transpose();
    // LOG(INFO) << "M = " << endl << M;
    // LOG(INFO) << "C = " << endl << C;
    // LOG(INFO) << "G = " << endl << G;
    // LOG(INFO) << "Psi = " << endl << Psi;
    // LOG(INFO) << "PsiDot = " << endl << PsiDot;
    // LOG(INFO) << "F = " << endl << F;
    // LOG(INFO) << "xEq = " << endl << xEq;
    // LOG(INFO) << "u = " << endl << u;
    // LOG(INFO) << "U = " << endl << U;
    // LOG(INFO) << "D = " << endl << D;
    // LOG(INFO) << "Dv = " << endl << Dv;
    // LOG(INFO) << "lagMult = " << endl << lagMult;
    // LOG(INFO) << "ddq = " << endl << ddq;
    // LOG(INFO) << "dx = " << endl << dx;
    // exit(0);

    return dx;
}

} // namespace sim
} // namespace disneysimple
