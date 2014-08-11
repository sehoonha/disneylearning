/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "CGMulti.h"
#include <iostream>

#include <Eigen/Core>
#include "utils/CppCommon.h"

#include "tbb/parallel_reduce.h"
#include "tbb/blocked_range.h"

namespace libgp
{

struct SumGrad {
    Eigen::VectorXd value;
    SumGrad(int _dim) : dim(_dim) { value = Eigen::VectorXd::Zero(dim); }
    SumGrad( SumGrad& s, tbb::split ) { dim = s.dim; value = Eigen::VectorXd::Zero(dim); }
    
    void operator()( tbb::blocked_range< std::vector<GaussianProcess*>::iterator >& r) {
        Eigen::VectorXd temp = value;
        for( std::vector<GaussianProcess*>::iterator i = r.begin(); i != r.end(); ++i ) {
            GaussianProcess* gp = (*i);
            // cout << "@@" << endl;
            Eigen::VectorXd g = gp->log_likelihood_gradient();
            temp += g;
            // cout << "##" << endl;
        }
        value = temp;
    }
    void join( SumGrad& rhs ) {value += rhs.value;}
    int dim;
};

CGMulti::CGMulti()
{
}

CGMulti::~CGMulti()
{
}

double CGMulti::log_likelihood() {
    double ret = 0.0;
    for (int i = 0; i < gp_array.size(); i++) {
        ret += gp_array[i]->log_likelihood();
    }
    return ret;
}


Eigen::VectorXd CGMulti::log_likelihood_gradient() {
    // // Serial version
    // LOG(INFO) << "G";
    // Eigen::VectorXd ret;
    // for (int i = 0; i < gp_array.size(); i++) {
    //     Eigen::VectorXd g = gp_array[i]->log_likelihood_gradient();
    //     if (i == 0) {
    //         ret = g;
    //     } else {
    //         ret += g;
    //     }
    //     cout << i << " : " << g.transpose() << endl;
    // }
    // LOG(INFO) << "G.. done " << ret.transpose();

    // LOG(INFO) << "G parallel";
    int dim = gp_array[0]->covf().get_param_dim();
    SumGrad total( dim  );
    parallel_reduce(
        tbb::blocked_range< std::vector<GaussianProcess*>::iterator >(
            gp_array.begin(),
            gp_array.end()
            ),
        total );    
    // LOG(INFO) << "G parallel.. done " << total.value.transpose();


    return total.value;
}

Eigen::VectorXd CGMulti::get_loghyper() {
    return gp_array[0]->covf().get_loghyper();
}

void CGMulti::set_loghyper(const Eigen::VectorXd& x) {
    for (int i = 0; i < gp_array.size(); i++) {
        gp_array[i]->covf().set_loghyper(x);
    }
}

void CGMulti::maximize(std::vector<GaussianProcess*>& _gp_array, size_t n, bool verbose)
{
    this->gp_array = _gp_array;
    
    const double INT = 0.1; // don't reevaluate within 0.1 of the limit of the current bracket
    const double EXT = 3.0; // extrapolate maximum 3 times the current step-size
    const int MAX = 20;		// max 20 function evaluations per line search
    const double RATIO = 10;	// maximum allowed slope ratio
    const double SIG = 0.1, RHO = SIG/2;
    /* SIG and RHO are the constants controlling the Wolfe-
       Powell conditions. SIG is the maximum allowed absolute ratio between
       previous and new slopes (derivatives in the search direction), thus setting
       SIG to low (positive) values forces higher precision in the line-searches.
       RHO is the minimum allowed fraction of the expected (from the slope at the
       initial point in the linesearch). Constants must satisfy 0 < RHO < SIG < 1.
       Tuning of SIG (depending on the nature of the function to be optimized) may
       speed up the minimization; it is probably not worth playing much with RHO.
    */

    /* The code falls naturally into 3 parts, after the initial line search is
       started in the direction of steepest descent. 1) we first enter a while loop
       which uses point 1 (p1) and (p2) to compute an extrapolation (p3), until we
       have extrapolated far enough (Wolfe-Powell conditions). 2) if necessary, we
       enter the second loop which takes p2, p3 and p4 chooses the subinterval
       containing a (local) minimum, and interpolates it, unil an acceptable point
       is found (Wolfe-Powell conditions). Note, that points are always maintained
       in order p0 <= p1 <= p2 < p3 < p4. 3) compute a new search direction using
       conjugate gradients (Polack-Ribiere flavour), or revert to steepest if there
       was a problem in the previous line-search. Return the best value so far, if
       two consecutive line-searches fail, or whenever we run out of function
       evaluations or line-searches. During extrapolation, the "f" function may fail
       either with an error or returning Nan or Inf, and maxmize should handle this
       gracefully.
    */


    bool ls_failed = false;									//prev line-search failed
    double f0 = -log_likelihood();						//initial negative marginal log likelihood
    Eigen::VectorXd df0 = -log_likelihood_gradient();	//initial gradient
    Eigen::VectorXd X = get_loghyper();			//hyper parameters

    if(verbose) LOG_EVERY_N(INFO, 50) << f0 << " <- " << get_loghyper().transpose();

    Eigen::VectorXd s = -df0;								//initial search direction
    double d0 = -s.dot(s);									//initial slope
    double x3 = 1/(1-d0);

    double f3 = 0;
    double d3 = 0;
    Eigen::VectorXd df3 = df0;

    double x2 = 0, x4 = 0;
    double f2 = 0, f4 = 0;
    double d2 = 0, d4 = 0;

    for (unsigned int i = 0; i < n; ++i)
    {
        //copy current values
        Eigen::VectorXd X0 = X;
        double F0 = f0;
        Eigen::VectorXd dF0 = df0;
        unsigned int M = std::min(MAX, (int)(n-i));

        while(1)											//keep extrapolating until necessary
        {
            x2 = 0;
            f2 = f0;
            d2 = d0;
            f3 = f0;
            df3 = df0;
            double success = false;

            while( !success && M>0)
            {
                M --;
                i++;
                set_loghyper(X+s*x3);
                f3 = -log_likelihood();
                df3 = -log_likelihood_gradient();

                if(verbose) LOG_EVERY_N(INFO, 50) << i << " : " << f3 << " <- " << get_loghyper().transpose() << endl;

                bool nanFound = false;
                //test NaN and Inf's
                for (int j = 0; j < df3.rows(); ++j)
                {
                    if(std::isnan(df3(j)))
                    {
                        nanFound = true;
                        break;
                    }
                }
                if(!std::isnan(f3) && !std::isinf(f3) && !nanFound)
                    success = true;
                else
                {
                    x3 = (x2+x3)/2; 						// if fail, bissect and try again
                }
            }
            //keep best values
            if(f3 < F0)
            {
                X0 = X+s*x3;
                F0 = f3;
                dF0 = df3;
            }

            d3 = df3.dot(s);								// new slope

            if( (d3 > SIG*d0) || (f3 >  f0+x3*RHO*d0) || M == 0) // are we done extrapolating?
            {
                break;
            }

            double x1 = x2; double f1 = f2; double d1 = d2;	// move point 2 to point 1
            x2 = x3; f2 = f3; d2 = d3;						// move point 3 to point 2
            double A = 6*(f1-f2) + 3*(d2+d1)*(x2-x1);				// make cubic extrapolation
            double B = 3*(f2-f1) - (2*d1+d2)*(x2-x1);
            x3 = x1-d1*(x2-x1)*(x2-x1)/(B+sqrt(B*B -A*d1*(x2-x1)));
            if(std::isnan(x3) || x3 < 0 || x3 > x2*EXT)			// num prob | wrong sign | beyond extrapolation limit
                x3 = EXT*x2;
            else if(x3 < x2+INT*(x2-x1))					// too close to previous point
                x3 = x2+INT*(x2-x1);
        }

        while( ( (abs(d3) > -SIG*d0) || (f3 > f0+x3*RHO*d0) ) && (M > 0))	// keep interpolating
        {
            if( (d3 > 0) || (f3 > f0+x3*RHO*d0) )			// choose subinterval
            {												// move point 3 to point 4
                x4 = x3;
                f4 = f3;
                d4 = d3;
            }
            else
            {
                x2 = x3;									//move point 3 to point 2
                f2 = f3;
                d2 = d3;
            }

            if(f4 > f0)
                x3 = x2 - (0.5*d2*(x4-x2)*(x4-x2))/(f4-f2-d2*(x4-x2));	// quadratic interpolation
            else
            {
                double A = 6*(f2-f4)/(x4-x2)+3*(d4+d2);
                double B = 3*(f4-f2)-(2*d2+d4)*(x4-x2);
                x3 = x2+sqrt(B*B-A*d2*(x4-x2)*(x4-x2) -B)/A;
            }

            if(std::isnan(x3) || std::isinf(x3))
                x3 = (x2+x4)/2;

            x3 = std::max(std::min(x3, x4-INT*(x4-x2)), x2+INT*(x4-x2));

            set_loghyper(X+s*x3);
            f3 = -log_likelihood();
            df3 = -log_likelihood_gradient();

            if(f3 < F0)												// keep best values
            {
                X0 = X+s*x3;
                F0 = f3;
                dF0 = df3;
            }

            if(verbose) LOG_EVERY_N(INFO, 50) << i << " : " << F0 << " <- " << get_loghyper().transpose() << endl;

            M--;
            i++;
            d3 = df3.dot(s);										// new slope
        }

        if( (abs(d3) < -SIG*d0) && (f3 < f0+x3*RHO*d0))
        {
            X = X+s*x3;
            f0 = f3;
            s = (df3.dot(df3)-df0.dot(df3)) / (df0.dot(df0))*s - df3;	// Polack-Ribiere CG direction
            df0 = df3;													// swap derivatives
            d3 = d0; d0 = df0.dot(s);
            if(verbose) LOG_EVERY_N(INFO, 50) << i << " : " << f0 << " <- " << get_loghyper().transpose() << endl;
            if(d0 > 0)													// new slope must be negative
            {															// otherwise use steepest direction
                s = -df0;
                d0 = -s.dot(s);
            }

            x3 = x3 * std::min(RATIO, d3/(d0-std::numeric_limits< double >::min()));	// slope ratio but max RATIO
            ls_failed = false;																// this line search did not fail
        }
        else
        {														// restore best point so far
            X = X0;
            f0 = F0;
            df0 = dF0;

            if(verbose) LOG_EVERY_N(INFO, 50) << i << " : " << f0 << " <- " << get_loghyper().transpose() << endl;

            if(ls_failed || i >= n)								// line search failed twice in a row
                break;											// or we ran out of time, so we give up

            s = -df0;
            d0 = -s.dot(s);										// try steepest
            x3 = 1/(1-d0);
            ls_failed = true;									// this line search failed
        }


    }
    set_loghyper(X);
}

}

