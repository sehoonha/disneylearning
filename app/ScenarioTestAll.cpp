/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "ScenarioTestAll.h"

#include <fstream>
#include <iomanip>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <tinyxml2.h>

#include "utils/CppCommon.h"
#include "utils/Misc.h"
#include "simulation/Manager.h"
#include "simulation/Simulator.h"
#include "simulation/SimBox2D.h"
#include "simulation/Evaluator.h"
#include "learning/Policy.h"

#include "Application.h"
#include "Window.h"

namespace disney {
namespace app {

void ScenarioTestAll(Application* app) {
    disney::simulation::Manager* manager = app->manager();

    int NP = app->numPolicies();
    int NS = manager->numSimulators();
    
    LOG(INFO) << "# of policies = " << NP;
    LOG(INFO) << "# of simulators = " << NS;

    Eigen::MatrixXd R(NP, NS);

    int maxSimLoop = app->maxSimLoop();
    LOG(INFO) << "maxSimLoop = " << maxSimLoop;

    std::vector< std::vector<simulation::SimulatorHistory> > allHistories(NS);

    for (int i = 0; i < NP; i++) {
        LOG(INFO) << "Evaluate policy " << i << " (" << app->nameOfPolicy(i) << ")";
        app->selectPolicy(i);
        app->reset();
        for (int j = 0; j < maxSimLoop; j++) { 
            app->step(); // step all simulators

        }

        for (int j = 0; j < NS; j++) {
            simulation::Simulator* s = manager->simulator(j);
            for (int k = 0; k < s->numHistories(); k++) {
                allHistories[j].push_back( s->history(k) );
            }

            double value = s->eval()->cost();
            R(i, j) = value;
        }
        LOG(INFO) << "Cost at row " << i << " : " << utils::V2S_SHORT(R.row(i));

        // LOG(INFO) << "attempt to collect simulator Box2D"; 
        // app->collectData(SIMTYPE_BOX2D);
        // LOG(INFO) << "attempt to collect simulator Box2D done."; 


    }

    for (int j = 0; j < NS; j++) {
        simulation::Simulator* s = manager->simulator(j);
        s->setAllHistories( allHistories[j] );
    }

    // LOG(INFO) << "Final result" << endl << R;
    LOG(INFO) << "Final result";
    std::ofstream fout("testall.csv");
    cout << std::fixed << std::setprecision(4);
    fout << std::fixed << std::setprecision(4);
    cout << "-";
    fout << "-";
    for (int j = 0; j < R.cols(); j++) {
        simulation::Simulator* s = manager->simulator(j);
        cout << ", " << s->type();
        fout << ", " << s->type();
    }
    cout << endl;
    fout << endl;

    for (int i = 0; i < R.rows(); i++) {
        cout << app->nameOfPolicy(i);
        fout << app->nameOfPolicy(i);
        for (int j = 0; j < R.cols(); j++) {
            cout << ", " << R(i, j);
            fout << ", " << R(i, j);
        }
        cout << endl;
        fout << endl;
    }
    fout.close();
}

void ScenarioTestAllParams(Application* app) {
    disney::simulation::Manager* manager = app->manager();

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(DATA_DIR"/testparams.xml");
    if (result != tinyxml2::XML_NO_ERROR) {
        doc.PrintError();
        LOG(FATAL) << "ErrorStr1: " << doc.GetErrorStr1();
        LOG(FATAL) << "ErrorStr2: " << doc.GetErrorStr2();
        exit(0);
    }

    int NDIM = 0;
    std::vector<Eigen::VectorXd> testparams;
    LOG(INFO) << "Load okay";
    tinyxml2::XMLNode* xnRoot = dynamic_cast<tinyxml2::XMLNode*>(&doc);
    tinyxml2::XMLElement* xnTest = xnRoot->FirstChildElement("test");
    for (tinyxml2::XMLElement* xnPolicy = xnTest->FirstChildElement("policy"); xnPolicy;
         xnPolicy = xnPolicy->NextSiblingElement("policy")) {
        std::string str_params = xnPolicy->Attribute("params");

        std::vector<std::string> tokens;
        boost::split(tokens, str_params, boost::is_any_of(", "));
        std::vector<double> vec_double;
        FOREACH(const std::string& s, tokens) {
            if (s.length() == 0) continue;
            vec_double.push_back( boost::lexical_cast<double>(s) );
        }
        Eigen::VectorXd params(vec_double.size());
        for (int i = 0; i < vec_double.size(); i++) {
            params(i) = vec_double[i];
        }


        NDIM = params.size();
        LOG(INFO) << "Policy: " << params.transpose();
        testparams.push_back(params);
    }

    int NP = testparams.size();
    int NS = manager->numSimulators();

    LOG(INFO) << "# of simulators = " << NS;
    LOG(INFO) << "# of policies = " << NP;
    LOG(INFO) << "# of dimensions = " << NDIM;


    Eigen::MatrixXd R(NP, NS);

    int maxSimLoop = app->maxSimLoop();
    LOG(INFO) << "maxSimLoop = " << maxSimLoop;

    // Select the default policy
    app->selectPolicy(0);
    LOG(INFO) << "Select the default policy: " << app->nameOfPolicy(0);
    learning::Policy* p = app->policy();
    for (int i = 0; i < NP; i++) {
        Eigen::VectorXd params = testparams[i];
        LOG(INFO) << "Testing params " << utils::V2S(params, 4);
        p->setParams(params);
        app->reset();
        for (int j = 0; j < maxSimLoop; j++) {
            app->step();
        }

        for (int j = 0; j < NS; j++) {
            simulation::Simulator* s = manager->simulator(j);
            double value = s->eval()->cost();
            R(i, j) = value;
        }
        LOG(INFO) << "Cost at row " << i << " : " << utils::V2S_SHORT(R.row(i));
    }

    // LOG(INFO) << "Final result" << endl << R;
    LOG(INFO) << "Final result";
    std::stringstream sout;
    sout << std::fixed << std::setprecision(4);

    sout << "num"; 
    for (int j = 0; j < NDIM; j++) {
        sout << ", p" << j;
    }
    for (int j = 0; j < R.cols(); j++) {
        simulation::Simulator* s = manager->simulator(j);
        sout << ", " << s->type();
    }
    sout << endl;

    for (int i = 0; i < R.rows(); i++) {
        Eigen::VectorXd params = testparams[i];
        sout << i;
        for (int j = 0; j < NDIM; j++) {
            sout << ", " << params(j);
        }
        for (int j = 0; j < R.cols(); j++) {
            sout << ", " << R(i, j);
        }
        sout << endl;
    }

    LOG(INFO) << endl << sout.str();
    std::ofstream fout("testall.csv");
    fout << sout.str();
    fout.close();
}



} // namespace app
} // namespace disney


