#ifndef GUI_GLWIDGET_H
#define GUI_GLWIDGET_H

#include <QGLWidget>
#include "utils/HppCommon.h"

#define GL_WINDOW_WIDTH  1280
#define GL_WINDOW_HEIGHT 720

namespace disney {
namespace app {

class Window;
class Camera;

} // namespace app
} // namespace disneylearning

namespace disney {
namespace app {
    
class GLWidget : public QGLWidget {
    Q_OBJECT;
public:
    GLWidget(Window* _parent);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL( int w, int h );

    void enable2D();
    void disable2D();

public slots:
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);

protected:
    MEMBER_PTR(Window*, window);
    MEMBER_PTR(Camera*, camera);
};

} // namespace app
} // namespace disney

#endif // #ifndef GUI_GLWIDGET_H
