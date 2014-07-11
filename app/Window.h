#ifndef GUI_WINDOW_H
#define GUI_WINDOW_H

#include <QWidget>
#include <QtGui>
#include <vector>
#include <map>
#include <string>

#include "utils/HppCommon.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QWidget;
class QComboBox;
QT_END_NAMESPACE

namespace disney {
namespace app {
class GLWidget;
class Application;
} // namespace app
} // namespace disney


namespace disney {
namespace app {
    
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
    bool isActionChecked(const char* const _name) { return actions[_name]->isChecked(); }
    void createToolbars();
    void createMenus();
                      

public slots:
    void onTimerRender();
    void onTimerIdle();
    void onSliderFrameChanged(int index);
    void onActionReset();
    void onActionPlay();
    void onActionStep();
    void onActionLoad();

    void onActionTrain();
    void onActionPause();
    void onActionCollect();
    void onActionConsume();
    void onActionOptimize();

    void onActionLoadHistory();
    void onMenuPolicy(QAction* action);

protected:
    void keyPressEvent(QKeyEvent* event);


    void takeCapture();
    void takeScreenshot(const char* const filename);
protected:
    MEMBER_PTR(Application*, app);
    // MEMBER_PTR(sim::Box2dSimulation*, boxsim);

    MEMBER_PTR(GLWidget*, gl);
    MEMBER_PTR(QTimer*, timerRender);
    MEMBER_PTR(QTimer*, timerIdle);
    MEMBER_PTR(QStatusBar*, statusbar);

    MEMBER_PTR(QLabel*, labelTime);
    MEMBER_PTR(QSlider*, sliderFrame);
    MEMBER_PTR(QComboBox*, comboSim);

protected:
    std::map<std::string, QAction*> actions;
        
protected:
    MEMBER_VAR(bool, wasRunning);
};

} // namespace app
} // namespace disney

#endif // #ifndef GUI_WINDOW_H
