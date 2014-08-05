#include "Window.h"

#include <iomanip>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include "GLWidget.h"
#include "utils/CppCommon.h"
#include "Application.h"
#include "ScenarioTestAll.h"
#include "Scripts.h"
// #include "box2dsimulation.h"


namespace disney {
namespace app {
Window* g_window_instance = NULL;

Window::Window()
    : QMainWindow()
    , MEMBER_INIT_NULL(timerRender)
    , MEMBER_INIT_NULL(timerIdle)
    , MEMBER_INIT_NULL(gl)
{
    srand( (unsigned int) time (NULL) );
    initApp();
    initUI();
    initTimer();
    setCenter();
    set_wasRunning(true);
}

Window::~Window() {
    MEMBER_RELEASE_PTR(timerRender);
    MEMBER_RELEASE_PTR(timerIdle);
    MEMBER_RELEASE_PTR(gl);
}

void Window::initApp() {
    set_app(new Application());
    app()->init();
    // set_boxsim(new sim::Box2dSimulation());
    // set_app( new Application() );
    // app()->init(GL_WINDOW_WIDTH, GL_WINDOW_HEIGHT);
    // // set_commander( new Commander(app()) );
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::initUI() {
    QWidget* widget = new QWidget;
    setCentralWidget(widget);
    QHBoxLayout* layout = new QHBoxLayout();

    set_gl( new GLWidget(this) );
    layout->addWidget( gl() );

    widget->setLayout(layout);

    createActions();
    createToolbars();
    set_statusbar( statusBar() );
    statusbar()->showMessage("Hi there!");
    createMenus();

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::initTimer() {
    set_timerRender( new QTimer(this) );
    connect(timerRender(), SIGNAL(timeout()), this, SLOT(onTimerRender()));
    timerRender()->start(30);

    set_timerIdle( new QTimer(this) );
    connect(timerIdle(), SIGNAL(timeout()), this, SLOT(onTimerIdle()));
    timerIdle()->start(0);

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::setCenter() {
    QWidget* widget = this;
    QSize size = widget->sizeHint();
    QDesktopWidget* desktop = QApplication::desktop();
    int width = desktop->width();
    int height = desktop->height();
    int mw = size.width();
    int mh = size.height();

    int centerW = 0;
    int centerH = 0;
    widget->move(centerW, centerH);

    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::createActions() {
    // For tool bars
    createAction("Reset");
    createAction("Play")->setCheckable(true);
    createAction("Anim")->setCheckable(true);
    createAction("Capture")->setCheckable(true);
    createAction("Step");
    createAction("StopAtEnd")->setCheckable(true);
    actions["StopAtEnd"]->setChecked(true);

    createAction("Overlay")->setCheckable(true);
    actions["Overlay"]->setChecked(false);


    createAction("NN");
    createAction("Train");
    createAction("Pause")->setCheckable(true);
    actions["Pause"]->setChecked(false);
    createAction("Collect");
    createAction("Consume");
    createAction("Optimize");
    createAction("Load")->setShortcut( QKeySequence("Ctrl+L") );

    // For menus
    createAction("LoadHistory");
    createAction("TestAll");
    createAction("TestAllParams");
    createAction("VectorField");
    createAction("VectorField3D");
    createAction("Comparison");
}

QAction* Window::createAction(const char* _name) {
    std::string name(_name);
    std::string method = "1onAction" + name + "()"; // SLOT(macro)
    boost::erase_all(method, " ");
    // LOG(INFO) << "name = " << name << " method = " << method;
    QAction* action = new QAction(tr(name.c_str()), this);
    connect(action, SIGNAL(triggered()), this, method.c_str());
    actions[name] = action;
    return action;
}
    
void Window::createToolbars() {
    QToolBar* toolbar = addToolBar(tr("Playback"));
    set_labelTime( new QLabel(tr("----")) );

    toolbar->addWidget( labelTime() );
    toolbar->addAction( actions["Reset"] );
    toolbar->addAction( actions["Play"] );
    toolbar->addAction( actions["Anim"] );
    toolbar->addAction( actions["Step"] );
    toolbar->addAction( actions["Capture"] );
    toolbar->addAction( actions["StopAtEnd"] );

    set_sliderFrame( new QSlider(Qt::Horizontal) );
    sliderFrame()->setMaximumSize(QSize(300, 30));
    toolbar->addWidget( sliderFrame() );
    connect( sliderFrame(), SIGNAL(valueChanged(int)),
             this, SLOT(onSliderFrameChanged(int)) );

    toolbar->addAction( actions["Overlay"] );
    toolbar->addAction( actions["Load"] );
    toolbar->addAction( actions["Train"] );
    toolbar->addAction( actions["Pause"] );

    toolbar->addSeparator();

    set_comboSim( new QComboBox() );
    FOREACH(const std::string& type, app()->allSimulatorNames()) {
        comboSim()->addItem(type.c_str());
    }
    toolbar->addWidget(comboSim());

    toolbar->addAction( actions["Collect"] );
    toolbar->addAction( actions["Consume"] );
    toolbar->addAction( actions["Optimize"] );

    LOG(INFO) << FUNCTION_NAME() << " OK";

}


void Window::createMenus() {
    QMenu* menuFile = menuBar()->addMenu(tr("File"));
    menuFile->addAction( actions["LoadHistory"] );
     
    std::string prevName = "";
    QMenu* menuPolicy = menuBar()->addMenu(tr("Policy"));
    for (int i = 0; i < app()->numPolicies(); i++) {
        std::string name = app()->nameOfPolicy(i);
        QAction* action = new QAction(tr(name.c_str()), this);
        if (i < 10) {
            action->setShortcut( QKeySequence(QString("Ctrl+%1").arg(i) ) );
        }
        if (name.substr(0, 3) != prevName.substr(0, 3)) {
            menuPolicy->addSeparator();
        }
        menuPolicy->addAction(action);
        // if (name == "Zero") {
        //     menuPolicy->addSeparator();
        // }
        prevName = name;
    }
    connect(menuPolicy, SIGNAL(triggered(QAction*)), this, SLOT(onMenuPolicy(QAction*)));

    QMenu* menuScenario = menuBar()->addMenu(tr("Scripts"));
    menuScenario->addAction( actions["TestAll"] );
    menuScenario->addAction( actions["TestAllParams"] );
    menuScenario->addSeparator();
    menuScenario->addAction( actions["VectorField"] );
    menuScenario->addAction( actions["VectorField3D"] );
    menuScenario->addAction( actions["Comparison"] );

    LOG(INFO) << FUNCTION_NAME() << " OK";
}
    

void Window::onTimerRender() {
    gl()->updateGL();
    statusbar()->showMessage( app()->statusMessage().c_str() );
}

void Window::onTimerIdle() {
    g_window_instance = this;

    int maxSimLoop = app()->maxSimLoop();
    
    if (actions["Play"]->isChecked()) {
        app()->step();

        if (actions["StopAtEnd"]->isChecked() && app()->numMaximumHistory() == maxSimLoop + 1) {
            actions["Play"]->setChecked(false);
            onActionPlay();
            // LOG(INFO) << "evaluate = " << sim()->getCost();
        }

        if (actions["Capture"]->isChecked()) {
            takeCapture();
        }
    }

    int n = app()->numMaximumHistory();

    if (actions["Anim"]->isChecked()) {
        int i = sliderFrame()->value();
        i += 5;
        if (i >= n) {
            actions["Anim"]->setChecked(false);
        }
        sliderFrame()->setValue(i); // will invoke onSliderFrameChanged()
        if (actions["Capture"]->isChecked()) {
            takeCapture();
        }
    }

    // updateInfo
    // std::stringstream sout;
    // sout << std::fixed << std::setprecision(4);

    // sout << "State = ";
    // Eigen::VectorXd state = sim()->getState();
    // for (int i = 0; i < state.size(); i++) {
    //     if (i != 0) sout << ", ";
    //     sout << state(i);
    // }
    // sout << " Torque = ";
    // Eigen::VectorXd torque = sim()->getTorque();
    // for (int i = 0; i < torque.size(); i++) {
    //     if (i != 0) sout << ", ";
    //     sout << torque(i);
    // }
    // sout << " Result = " << sim()->getCost();
    
    // statusbar()->showMessage(sout.str().c_str());
}

void Window::onSliderFrameChanged(int index) {
    app()->updateToHistory(index);
    // sim()->updateToHistory(index);
    // boxsim()->updateToHistory(index);
}

void Window::onActionReset() {
    app()->reset();
    // sim()->reset();
    // boxsim()->reset();
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::onActionPlay() {
    int n = app()->numMaximumHistory();
    sliderFrame()->setRange(0, n - 1);

    if (actions["Play"]->isChecked()) {
        // sim()->updateToLatestHistory();
        // boxsim()->updateToLatestHistory();
        LOG(INFO) << FUNCTION_NAME() << " resumes";
    } else {
        LOG(INFO) << FUNCTION_NAME() << " pauses";
    }
}

void Window::onActionAnim() {
    int n = app()->numMaximumHistory();
    sliderFrame()->setRange(0, n - 1);
}

void Window::onActionStep() {
    // LOG(INFO) << FUNCTION_NAME() << " OK";
    // sim()->step();
    // boxsim()->step();
}

void Window::onActionLoad() {
    // QString qfilename = QFileDialog::getOpenFileName(
    //     this, tr("Load Neural Network from nn"), tr("./"), tr("NN Files (*.nn)"));
    // std::string filename = qfilename.toStdString();
    // if (filename.length() == 0) {
    //     LOG(WARNING) << "User cancelled loading";
    //     return;
    // }
    // LOG(INFO) << "Filename = [" << filename << "]";
    LOG(INFO) << FUNCTION_NAME() << " OK";
}


void Window::onActionTrain() {
    app()->train();
    
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::onActionPause() {
    int n = app()->numMaximumHistory();
    sliderFrame()->setRange(0, n - 1);


    app()->togglePause();
    LOG(INFO) << FUNCTION_NAME() << " OK : pause = " << pause;
}

void Window::onActionCollect() {
    app()->collectData(comboSim()->currentText().toStdString().c_str());
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::onActionConsume() {
    app()->consumeData();
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::onActionOptimize() {
    app()->optimizeGP();
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::onActionLoadHistory() {
    QString qfilename = QFileDialog::getOpenFileName(
        this, tr("Load Simulation History"), tr("./"), tr("CSV Files (*.csv)"));
    std::string filename = qfilename.toStdString();
    if (filename.length() == 0) {
        LOG(WARNING) << "User cancelled loading";
        return;
    }
    LOG(INFO) << "Filename = [" << filename << "]";
    app()->loadHistory(filename.c_str());
    LOG(INFO) << FUNCTION_NAME() << " OK";

    int n = app()->numMaximumHistory();
    sliderFrame()->setRange(0, n - 1);

}

void Window::onMenuPolicy(QAction* action) {
    std::string txt = action->text().toStdString();
    LOG(INFO) << FUNCTION_NAME() << " : " << txt;
    app()->selectPolicy( txt.c_str() );
}

void Window::onActionTestAll() {
    LOG(INFO) << FUNCTION_NAME();
    boost::thread t(&ScenarioTestAll, app());
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::onActionTestAllParams() {
    LOG(INFO) << FUNCTION_NAME();
    boost::thread t(&ScenarioTestAllParams, app());
    LOG(INFO) << FUNCTION_NAME() << " OK";
}


void Window::onActionVectorField() {
    LOG(INFO) << FUNCTION_NAME();
    plotVectorField(app());
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::onActionVectorField3D() {
    LOG(INFO) << FUNCTION_NAME();
    testVectorField3D(app());
    LOG(INFO) << FUNCTION_NAME() << " OK";
}

void Window::onActionComparison() {
    LOG(INFO) << FUNCTION_NAME();
    compareBox2DandMath(app());
    LOG(INFO) << FUNCTION_NAME() << " OK";
}



void Window::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_U) {
        // LOG(INFO) << FUNCTION_NAME() << " : U";
        // app()->up();
        return;
    }
    if (event->key() == Qt::Key_D) {
        // LOG(INFO) << FUNCTION_NAME() << " : D";
        // app()->down();
        return;
    }
    if (event->key() == Qt::Key_P) {
        // LOG(INFO) << FUNCTION_NAME() << " : P";
        // app()->perturb();
        return;
    }
}

void Window::gTakeCapture() {
    g_window_instance->takeCapture();
}


int captureRate = 0;
int captureID = 0;
void Window::takeCapture() {
    // if (!actions["Play"]->isChecked()) {
    //     return;
    // }
    // if (!actCapture()->isChecked()) {
    //     return;
    // }
    captureRate++;
    if (captureRate % 2 != 1) {
        return;
    }
        
    // std::string filename = (boost::format("./captures/capture%04d.png") % captureID).str();
    std::string filename = (boost::format("./captures/capture.%04d.png")
                            % captureID).str();
    takeScreenshot(filename.c_str());
    captureID++;

}

// avconv -r 100 -i ./capture.%04d.png output.mp4


void Window::takeScreenshot(const char* const filename) {
    // QPixmap pixmap;
    // pixmap = QPixmap::grabWindow(this->winId());
    // // pixmap = pixmap.scaled(1200, 800);
    // // pixmap.grabWidget(this->gl());
    // LOG(INFO) << "size = " << pixmap.width() << " x " << pixmap.height();

    // pixmap.save(tr(filename), "png");
    // QImage img(gl()->size(), QImage::Format_RGB32);
    // QPainter painter(&img);
    // painter.setPen(Qt::blue);
    // painter.setFont(QFont("Arial", 30));
    // painter.drawText(rect(), Qt::AlignCenter, "Qt");

    QImage img = gl()->grabFrameBuffer();
    img = img.convertToFormat(QImage::Format_ARGB32);
    QRect rect(0, 0, 1280, 720);
    QImage sub = img.copy(rect);
    img = sub;
    img.save(filename);
    LOG(INFO) << FUNCTION_NAME() << " : [" << filename << "]";
}

} // namespace app
} // namespace disney
