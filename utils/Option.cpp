/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "Option.h"
#include "CppCommon.h"
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <tinyxml2.h>

namespace disney {
namespace utils {

////////////////////////////////////////////////////////////
// struct OptionImp implementation
struct OptionImp {
    tinyxml2::XMLDocument* doc;    
    OptionItem parse(tinyxml2::XMLElement* element) {
        OptionItem ret;
        const char* const text = element->GetText();
        if (text) {
            ret.value = element->GetText();
        }
        // cout << "value = " << ret.value << endl;
        for (const tinyxml2::XMLAttribute* a = element->FirstAttribute();
             a != NULL; a = a->Next()) {
            std::string key   = a->Name();
            std::string value = a->Value();
            ret.attrs[key] = value;
        }
        return ret;
    }
};
// struct OptionImp ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// struct OptionItem implementation
double OptionItem::toDouble() const {
    return boost::lexical_cast<double>(value);
}

int OptionItem::toInt() const {
    return boost::lexical_cast<int>(value);
}

bool OptionItem::toBool() const {
    return boost::lexical_cast<bool>(value);
}

Eigen::VectorXd OptionItem::toVectorXd() const {
    std::vector<std::string> vec_string;
    boost::split(vec_string, value, boost::is_any_of(", "));
    std::vector<double> vec_double;
    FOREACH(const std::string& s, vec_string) {
        if (s.length() == 0) continue;
        vec_double.push_back( boost::lexical_cast<double>(s) );
    }
    Eigen::VectorXd ret(vec_double.size());
    for (int i = 0; i < vec_double.size(); i++) {
        ret(i) = vec_double[i];
    }
    return ret;
}

bool OptionItem::hasAttr(const char* const key) const {
    return (attrs.find(key) != attrs.end());
}

double OptionItem::attrDouble(const char* const key) const {
    // return boost::lexical_cast<double>(attrs[key]);
    return boost::lexical_cast<double>(attrs.at(key));
}

int OptionItem::attrInt(const char* const key) const {
    return boost::lexical_cast<int>(attrs.at(key));
}

bool OptionItem::attrBool(const char* const key) const {
    return boost::lexical_cast<bool>(attrs.at(key));
}
    
std::string OptionItem::attrString(const char* const key) const {
    return attrs.at(key);
}

std::vector<double> OptionItem::attrVectorDouble(const char* const key) const {
    std::vector<std::string> vec_string;
    boost::split(vec_string, attrs.at(key), boost::is_any_of(", "));
    std::vector<double> vec_double;
    FOREACH(const std::string& s, vec_string) {
        if (s.length() == 0) continue;
        vec_double.push_back( boost::lexical_cast<double>(s) );
    }
    return vec_double;
}

// struct OptionImp ends
////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
// class Option implementation
Option* Option::g_instance = NULL;

//// static public

bool Option::init(const char* const filename) {
    if (g_instance) {
        destroy();
    }
    g_instance = new Option();
    bool ret = g_instance->load(filename);
    LOG(INFO) << FUNCTION_NAME() << " OK";
    return ret;
}

bool Option::destroy() {
    if (g_instance == NULL) {
        return false;
    }

    // g_instance is not NULL
    delete g_instance;
    g_instance = NULL;
    LOG(INFO) << FUNCTION_NAME();
    return true;
}

OptionItem Option::read(const char* const option) {
    Option* opt = g_instance;
    std::vector<std::string> keys = opt->split(option);
    int nextKeyIndex = 0;

    tinyxml2::XMLNode* xnCurr = dynamic_cast<tinyxml2::XMLNode*>(opt->imp->doc);
    CHECK_NOTNULL(xnCurr);

    for (int i = 0; i < keys.size(); i++) {
        const std::string& key = keys[i];
        tinyxml2::XMLNode* child = xnCurr->FirstChildElement(key.c_str());
        if (child == NULL) {
            break;
        }
        xnCurr = child;
        nextKeyIndex = i + 1;
    }

    tinyxml2::XMLElement* xnFinalNode = dynamic_cast<tinyxml2::XMLElement*>(xnCurr);
    CHECK_NOTNULL(xnFinalNode);

    OptionItem ret;
    // OptionItem == Attribute
    if (nextKeyIndex < keys.size()) { 
        std::string key = keys[nextKeyIndex];
        ret.value = xnFinalNode->Attribute(key.c_str());
    // OptionItem == Element
    } else {
        ret = opt->imp->parse(xnFinalNode);
    }

    return ret;
}

std::vector<OptionItem> Option::readAll(const char* const option) {
    std::vector<OptionItem> ret;
    Option* opt = g_instance;
    std::vector<std::string> keys = opt->split(option);
    std::string lastKey;

    tinyxml2::XMLNode* xnCurr = dynamic_cast<tinyxml2::XMLNode*>(opt->imp->doc);
    CHECK_NOTNULL(xnCurr);

    for (int i = 0; i < keys.size(); i++) {
        const std::string& key = keys[i];
        tinyxml2::XMLNode* child = xnCurr->FirstChildElement(key.c_str());
        if (child == NULL) {
            LOG(WARNING) << "cannot find key [" << key << "] in " << option;
            return ret;
        }
        lastKey = key;
        xnCurr = child;
    }
    
    tinyxml2::XMLElement* xnFinalNode = dynamic_cast<tinyxml2::XMLElement*>(xnCurr);
    CHECK_NOTNULL(xnFinalNode);
    for (tinyxml2::XMLElement* node = xnFinalNode;
         node; node = node->NextSiblingElement(lastKey.c_str())) {
        OptionItem item = opt->imp->parse(node);
        ret.push_back(item);
    }

    return ret;
}

//// protected

Option::Option() {
    imp = new OptionImp;
    imp->doc = NULL;
}

Option::~Option() {
    if (imp) {
        if (imp->doc) delete imp->doc;
        delete imp;
    }
}

std::vector<std::string> Option::split(const char* const option) {
    std::vector<std::string> ret;
    boost::split(ret, option, boost::is_any_of("."));
    return ret;
}

bool Option::load(const char* const filename) {
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();
    tinyxml2::XMLError result
        = doc->LoadFile(filename);
    if (result != tinyxml2::XML_NO_ERROR) {
        doc->PrintError();
        LOG(FATAL) << "ErrorStr1: " << doc->GetErrorStr1();
        LOG(FATAL) << "ErrorStr2: " << doc->GetErrorStr2();
        return false;
    }    
    imp->doc = doc;
    return true;
}



// class Option ends
////////////////////////////////////////////////////////////


} // namespace utils
} // namespace disney


