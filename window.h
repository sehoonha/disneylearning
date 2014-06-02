#ifndef GUI_WINDOW_H
#define GUI_WINDOW_H

#include <QWidget>
#include <QtGui>
#include <vector>
#include <map>
#include <string>

#include "hppcommon.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QWidget;
QT_END_NAMESPACE

namespace disneysimple {
namespace gui {
class GLWidget;
// class Commander;
} // namespace gui
namespace sim {
class Simulation;
} // namespace sim
} // namespace disneysimple


namespace disneysimple {
namespace gui {
    
class Window : public QMainWindow {
    Q_OBJECT;

public:
    Window();
    virtual ~Window();
        
    void initApp();
    void initUI();
    void initTimer();
    void setCenter();

    void createActions();
    QAction* createAction(const char* _name);
    void createToolbars();
    void createMenus();

public slots:
    void onTimerRender();
    void onTimerIdle();
    void onSliderFrameChanged(int index);
    void onActionPlay();
    void onActionLoad();

protected:
    void keyPressEvent(QKeyEvent* event);


    void takeCapture();
    void takeScreenshot(const char* const filename);
protected:
    MEMBER_PTR(sim::Simulation*, sim);

    MEMBER_PTR(GLWidget*, gl);
    MEMBER_PTR(QTimer*, timerRender);
    MEMBER_PTR(QTimer*, timerIdle);
    MEMBER_PTR(QStatusBar*, statusbar);

    MEMBER_PTR(QLabel*, labelTime);
    MEMBER_PTR(QSlider*, sliderFrame);

protected:
    std::map<std::string, QAction*> actions;
        
protected:
    MEMBER_VAR(bool, wasRunning);
};

} // namespace gui
} // namespace disneysimple

#endif // #ifndef GUI_WINDOW_H
