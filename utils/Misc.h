/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef UTILS_MISC_H
#define UTILS_MISC_H

#include <sstream>
#include <iomanip>
#include <Eigen/Dense>

namespace disney {
namespace utils {

static std::string V2S(const Eigen::VectorXd& v, int precision = 10) {
    std::stringstream sout;
    sout << std::fixed << std::setprecision(precision);
    for (int i = 0; i < v.size(); i++) {
        if (i > 0) sout << ", ";
        sout << v(i);
    }
    return sout.str();
}



static std::string V2S_SHORT(const Eigen::VectorXd& v) {
    std::stringstream sout;
    sout << std::fixed << std::setprecision(2);
    for (int i = 0; i < v.size(); i++) {
        if (i > 0) sout << ", ";
        sout << v(i);
    }
    return sout.str();
}

inline double random_uniform(double _min, double _max) {
  return _min + ((static_cast<double>(rand()) / (RAND_MAX + 1.0))
                * (_max - _min));
}

} // namespace utils
} // namespace disney

#endif // #ifndef UTILS_MISC_H

