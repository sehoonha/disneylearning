#ifndef APP_CPPCOMMON_H
#define APP_CPPCOMMON_H

#include <iostream>
using std::cout;
using std::endl;

////////////////////////////////////////////////////////////
// Boost Libraries
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>

using namespace boost::assign;
#define FOREACH         BOOST_FOREACH
#define REVERSE_FOREACH BOOST_REVERSE_FOREACH
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// Google Libraries
#include <glog/logging.h>
using google::INFO;
using google::WARNING;
using google::ERROR;
using google::FATAL;
// #include <gflags/gflags.h>
// #define CHECK_NOTREACHED() do { LOG(FATAL) << "Should not be reached"; } while (false)
// ////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////
// // Boost logging
// #define LOGGING_BOOST
// #include <boost/log/trivial.hpp>
// #include <boost/log/expressions.hpp>
// #include <boost/log/sources/severity_logger.hpp>
// #include <boost/log/sources/record_ostream.hpp>
// #include <boost/log/utility/setup/console.hpp>
// #include <boost/log/utility/setup/file.hpp>
// #include <boost/log/utility/setup/common_attributes.hpp>
// #include <boost/log/support/date_time.hpp>

// #define LOG_INFO BOOST_LOG_TRIVIAL(info)
// #define LOG_WARNING BOOST_LOG_TRIVIAL(warning)
// #define LOG_ERROR BOOST_LOG_TRIVIAL(error)
// #define LOG_FATAL BOOST_LOG_TRIVIAL(fatal)
// #define CHECK_EQ(lhs, rhs) if ((lhs) != (rhs)) BOOST_LOG_TRIVIAL(fatal)
// #define CHECK_LT(lhs, rhs) if ((lhs) >= (rhs)) BOOST_LOG_TRIVIAL(fatal)
// #define CHECK_NOTNULL(ptr) if ((ptr) == NULL) BOOST_LOG_TRIVIAL(fatal)
// #define VLOG_INFO if (verbose) BOOST_LOG_TRIVIAL(info)

// ////////////////////////////////////////////////////////////
// // std logging
// #define LOG_INFO if (false) std::cout
// #define LOG_WARNING std::cerr
// #define LOG_ERROR std::cerr
// #define LOG_FATAL std::cerr
// #define CHECK_EQ(lhs, rhs) if ((lhs) != (rhs)) std::cerr
// #define CHECK_LT(lhs, rhs) if ((lhs) >= (rhs)) std::cerr
// #define CHECK_NOTNULL(ptr) if ((ptr) == NULL) std::cerr
// #define VLOG_INFO if (verbose) std::cout


////////////////////////////////////////////////////////////
// Simple Macros
#ifdef __linux
#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
#elif defined(__APPLE__)
#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
//#error "Define your own FUNCTION_NAME() macro on Apple"
#elif defined(_WIN32)
// #define FUNCTION_NAME() (__FUNCSIG__)
#define FUNCTION_NAME() (__FUNCTION__)
#else
#error "What's your operating system?"
#endif
////////////////////////////////////////////////////////////

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CONFINE(a, lo, hi) (((a)<(lo))?(lo):(  (((a)>(hi))?(hi):(a))           ))

#ifndef PI
#define PI 3.141592653589793
#endif
#ifndef PI_2 
#define PI_2 1.5707963267948966
#endif

// #include "Paths.h"

#endif // #ifndef APP_CPPCOMMON_H
