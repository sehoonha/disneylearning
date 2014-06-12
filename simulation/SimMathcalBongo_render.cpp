/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "SimMathcalBongo.h"

#include "utils/LoadOpengl.h"
#include "utils/CppCommon.h"

namespace disney {
namespace simulation {

// !!! Becareful about the order or arguments!!
void line(double x1, double x2, double y1, double y2) {
    glBegin(GL_LINES);
    glVertex2d(x1, y1);
    glVertex2d(x2, y2);
    glEnd();
}


void circle(double cx, double cy, double r) {
    glBegin(GL_POLYGON);
    for (double th = 0.0; th < 2 * PI; th += (PI / 10.0) ) {
        double x = cx + r * cos(th);
        double y = cy + r * sin(th);
        glVertex2d(x, y);
    }
    glEnd();
}

void SimMathcalBongo::render() {
    glPushMatrix();
    glTranslated(1.0, 0.0, 0.0);

    // glBegin(GL_POLYGON);
    // glVertex2d(0, 0);
    // glVertex2d(0, 0.1);
    // glVertex2d(0.1, 0.1);
    // glEnd();

    // Parameters
    double offset = 0;
    double radius = 0.05;
    double rw     = radius;
    double aXmin  = -0.6;
    double aXmax  = 0.6;
    double aYmin  = 2*rw-0.3;
    double aYmax  = 2*rw+1.5;
    double lb     = 0.6;

    double lrl1 = 1;
    double lrl2 = 0.1;
    double lll1 = 1;
    double lll2 = 0.1;

    // Fetch the state
    int n = numDimConfig();
    Eigen::VectorXd q = mState.head(n);
    double alphaw  = q(0);
    double alphab  = q(1);
    double al      = 0.0;
    double ar      = 0.0;
    double thetal1 = q(2);
    double thetar1 = q(3);
    double thetal2 = q(4);
    double thetar2 = q(5);

    // wheel position
    Eigen::Vector3d wheel;
    wheel << 0,
        offset-alphaw*rw,
        rw;


    // board end positions
    Eigen::Vector3d boardCOM;
    boardCOM << 0,
        offset-rw*(alphaw + sin(alphab + alphaw) - alphab*cos(alphab + alphaw)),
        2*rw*(cos(alphab + alphaw)/2 + 1/2) + alphab*rw*sin(alphab + alphaw);

    Eigen::Vector3d boardRight;
    boardRight << 0,
        offset- alphaw*rw - cos(alphab + alphaw)*(lb/2 - alphab*rw) - rw*sin(alphab + alphaw),
        rw - sin(alphab + alphaw)*(lb/2 - alphab*rw) + rw*cos(alphab + alphaw);

    Eigen::Vector3d boardLeft;
    boardLeft << 0,
        offset+cos(alphab + alphaw)*(lb/2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw),
        rw + sin(alphab + alphaw)*(lb/2 + alphab*rw) + rw*cos(alphab + alphaw);     

    
    // feet cart positions
    Eigen::Vector3d rightCart;
    rightCart << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - rw*sin(alphab + alphaw) - alphaw*rw,
        rw + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw);
    
    Eigen::Vector3d leftCart;
    leftCart << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw),
        rw + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw)  ;          

    // link1 com positions
    Eigen::Vector3d rightLink1COM;
    rightLink1COM << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - rw*sin(alphab + alphaw) - alphaw*rw - (lrl1*sin(alphab + alphaw + thetar1))/2,
        rw + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw) + (lrl1*cos(alphab + alphaw + thetar1))/2  ;
    
    Eigen::Vector3d leftLink1COM;
    leftLink1COM << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw) - (lll1*sin(alphab + alphaw + thetal1))/2,
        rw + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw) + (lll1*cos(alphab + alphaw + thetal1))/2  ;

    // link1 end positions
    Eigen::Vector3d rightLink1;
    rightLink1 << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - rw*sin(alphab + alphaw) - alphaw*rw - lrl1*sin(alphab + alphaw + thetar1),
        rw + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw) + lrl1*cos(alphab + alphaw + thetar1)   ;
    
    Eigen::Vector3d leftLink1;
    leftLink1 << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw) - lll1*sin(alphab + alphaw + thetal1),
        rw + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw) + lll1*cos(alphab + alphaw + thetal1)  ;
    
    // link2 positions
    Eigen::Vector3d leftLink2;
    leftLink2 << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - lll2*sin(alphab + alphaw + thetal1 + thetal2) - rw*sin(alphab + alphaw) - lll1*sin(alphab + alphaw + thetal1),
        rw + lll2*cos(alphab + alphaw + thetal1 + thetal2) + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw) + lll1*cos(alphab + alphaw + thetal1)  ;
    
    Eigen::Vector3d rightLink2;
    rightLink2 << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - lrl2*sin(alphab + alphaw + thetar1 + thetar2) - rw*sin(alphab + alphaw) - alphaw*rw - lrl1*sin(alphab + alphaw + thetar1),
        rw + lrl2*cos(alphab + alphaw + thetar1 + thetar2) + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw) + lrl1*cos(alphab + alphaw + thetar1)  ;

    Eigen::Vector3d cart = 0.5 * (leftCart + rightCart);
    
    glLineWidth(2);
    const int X = 1;
    const int Y = 2;
    glColor3d(1, 0, 0);
    circle( wheel(X), wheel(Y), rw);
    glColor3d(1, 1, 0);
    line(boardLeft(X), boardRight(X), boardLeft(Y), boardRight(Y));
    glColor3d(0, 1, 0);
    line(rightCart(X), rightLink1(X), rightCart(Y), rightLink1(Y));
    line(rightLink1(X), rightLink2(X), rightLink1(Y), rightLink2(Y));
    glColor3d(0, 0, 1);
    line(leftCart(X), leftLink1(X), leftCart(Y), leftLink1(Y));
    line(leftLink1(X), leftLink2(X), leftLink1(Y), leftLink2(Y));
    glColor3d(0, 0, 0);
    line(-3, 3, 0, 0);

    glColor3d(0, 1, 1);
    circle( rightLink2(X), rightLink2(Y), 0.02 );
    circle( cart(X), cart(Y), 0.02 );

    glPopMatrix();
}


} // namespace simulation
} // namespace disney


