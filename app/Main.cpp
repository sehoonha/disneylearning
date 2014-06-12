#include <QApplication>
#include <QIcon>
#include "Window.h"
#include "utils/CppCommon.h"
#include "utils/LoadOpengl.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>

int main(int argc, char *argv[])
{
    // google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging((const char*)argv[0]);
    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = INFO;
    FLAGS_log_dir = "./glog/";

    // Init GLUT
    glutInit(&argc, argv);

    // Init and launch the application
    QApplication app(argc, argv);
    QIcon icon("icon.png");
    app.setWindowIcon(icon);

    disney::gui::Window window;
    window.show();
    window.activateWindow();
    window.raise();
    return app.exec();
}
