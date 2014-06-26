/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#ifndef UTILS_OPTION_H
#define UTILS_OPTION_H

#include <string>
#include <vector>
#include <map>

namespace disney {
namespace utils {

struct OptionItem;
class Option;
struct OptionImp;

struct OptionItem {
    std::string value;
    std::map< std::string, std::string > attrs;
    double toDouble() const;

    double attrDouble(const char* const key) const;
    std::string attrString(const char* const key) const;
};


class Option {
protected:
    static Option* g_instance;
public:
    static bool init(const char* const filename);
    static bool destroy();

    static OptionItem read(const char* const option);
    static std::vector<OptionItem> readAll(const char* const option);
protected:
    Option();
    virtual ~Option();
    std::vector<std::string> split(const char* const option);
    bool load(const char* const filename);
    OptionImp* imp;
}; // class Option

} // namespace utils
} // namespace disney

#endif // #ifndef UTILS_OPTION_H

