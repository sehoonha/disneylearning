#include "Window.h"

#include <iomanip>
#include <boost/algorithm/string.hpp>

#include "GLWidget.h"
#include "utils/CppCommon.h"
#include "Application.h"
// #include "box2dsimulation.h"

namespace disney {
namespace app {
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
    createAction("Reset");
    createAction("Play")->setCheckable(true);
    createAction("Anim")->setCheckable(true);
    createAction("Capture")->setCheckable(true);
    createAction("Step");
    createAction("StopAtEnd")->setCheckable(true);


    createAction("NN");
    createAction("Train");
    createAction("Load")->setShortcut( QKeySequence("Ctrl+L") );
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

    toolbar->addAction( actions["NN"] );
    toolbar->addAction( actions["Load"] );
    toolbar->addAction( actions["Train"] );

    LOG(INFO) << FUNCTION_NAME() << " OK";

}


void Window::createMenus() {
    QMenu* menuFile = menuBar()->addMenu(tr("File"));
    menuFile->addAction( actions["Load"] );
     
    LOG(INFO) << FUNCTION_NAME() << " OK";
}
    

void Window::onTimerRender() {
    gl()->updateGL();
}

void Window::onTimerIdle() {
    if (actions["Play"]->isChecked()) {
        app()->step();

        // if (actions["StopAtEnd"]->isChecked() && sim()->getTime() >= 4.99999) {
        //     actions["Play"]->setChecked(false);
        //     // LOG(INFO) << "evaluate = " << sim()->getCost();
        // }

        if (actions["Capture"]->isChecked()) {
            takeCapture();
        }
    }

    int n = app()->numMaximumHistory();

    if (actions["Anim"]->isChecked()) {
        int i = sliderFrame()->value();
        i++;
        if (i >= n) {
            i = 0;
        }
        sliderFrame()->setValue(i); // will invoke onSliderFrameChanged()
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

void Window::onActionStep() {
    // LOG(INFO) << FUNCTION_NAME() << " OK";
    // sim()->step();
    // boxsim()->step();
}

void Window::onActionLoad() {
    QString qfilename = QFileDialog::getOpenFileName(
        this, tr("Load Neural Network from nn"), tr("./"), tr("NN Files (*.nn)"));
    std::string filename = qfilename.toStdString();
    if (filename.length() == 0) {
        LOG(WARNING) << "User cancelled loading";
        return;
    }
    LOG(INFO) << "Filename = [" << filename << "]";
    // sim()->loadNN(filename.c_str());
}

void Window::onActionNN() {
    LOG(INFO) << FUNCTION_NAME() << " OK";
    // sim()->loadNN();
}

void Window::onActionTrain() {
    // sim()->trainNN();
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
    if (captureRate % 10 != 1) {
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
