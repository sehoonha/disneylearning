#include <QApplication>
#include <QIcon>
#include "Window.h"
#include "utils/CppCommon.h"
#include "utils/LoadOpengl.h"
#include "utils/Option.h"

#include <string>
#include <csignal>
#include <cstdio>
#include <cstdlib>

#include "Application.h"

void runNoGui() {
    LOG(INFO) << "Run the application in NO GUI MODE!!!!!!";
    disney::app::Application* app = new disney::app::Application();
    app->init();
    app->train(false);
    LOG(INFO) << "Exit the application in NO GUI MODE!!!!!!";
    LOG(INFO) << "BYE!!!!!";

}

int main(int argc, char *argv[])
{
    // google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging((const char*)argv[0]);
    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = INFO;
    FLAGS_log_dir = "./glog/";


    // Init Option
    disney::utils::Option::init(DATA_DIR"/setting.xml");
    // cout << disney::utils::Option::read("simulation.init.angle").toDouble() << endl;
    // cout << disney::utils::Option::read("simulation.init").attrDouble("angle") << endl;
    // std::vector<disney::utils::OptionItem> opts = disney::utils::Option::readAll("simulation.sim");
    // FOREACH(disney::utils::OptionItem& o, opts) {
    //     cout << "Sim: " << o.attrString("type") << endl;
    // }

    if (argc >= 2) {
        std::string arg1 = argv[1];
        LOG(INFO) << "Argument 1: " << arg1;
        if (arg1 == "nogui") {
            runNoGui();
            return 0;
        }
    }
        

    // Init GLUT
    glutInit(&argc, argv);


    // Init and launch the application
    QApplication app(argc, argv);
    QIcon icon("icon.png");
    app.setWindowIcon(icon);

    disney::app::Window window;

    // Qt::WindowFlags eFlags = window.windowFlags();
    // eFlags |= Qt::WindowStaysOnTopHint;
    // window.setWindowFlags(eFlags);

    window.show();
    window.activateWindow();
    window.raise();
    
    return app.exec();
}
