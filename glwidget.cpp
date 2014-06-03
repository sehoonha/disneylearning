#include "glwidget.h"

#include "LoadOpengl.h"
#include "cppcommon.h"

// Local headers
#include "window.h"
#include "camera.h"
#include "simulation.h"

namespace disneysimple {
namespace gui {

GLWidget::GLWidget(Window* _parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers))
    , MEMBER_INIT(window, _parent)
    , MEMBER_INIT_NULL(camera)
{
    setFixedSize(GL_WINDOW_WIDTH, GL_WINDOW_HEIGHT);
    setAutoFillBackground(false);

    set_camera( new Camera() );

    camera()->begin = Eigen::Vector2d(0, 0);
    camera()->trackball(0.0f, 0.0f, 0.0f, 0.0f);

    camera()->pos  = Eigen::Vector3d(-0.8, -0.95, 5.3);

    camera()->q(0) = -0.07527;
    camera()->q(1) = -0.001988;
    camera()->q(2) = -0.000;
    camera()->q(3) = 0.99645;
}
                                 
void GLWidget::initializeGL() {
    static const GLfloat light0_pos[4]   = { 0.0f, 50.0f, 0.0f, 0.0f };
    // white light
    static const GLfloat light0_color[4] = { 0.01f, 0.01f, 0.01f, 1.0f };
    static const GLfloat light1_pos[4]   = {  50.0f, 50.0f, 0.0f, 0.0f };
    // cold blue light
    static const GLfloat light1_color[4] = { 0.4f, 0.4f, 0.4f, 1.0f };

    const char* vendor = (const char*)glGetString( GL_VENDOR );
    const char* renderer = (const char*)glGetString( GL_RENDERER );
    const char* version = (const char*)glGetString( GL_VERSION );
    const char* extensions = (const char*)glGetString( GL_EXTENSIONS );
    // cout << "Vendor = " << vendor << endl;
    // cout << "Renderer = " << renderer << endl;
    // cout << "Version = " << version << endl;
    // cout << "Extensions = " << extensions << endl;

    /* remove back faces */
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    // glEnable(GL_CULL_FACE);
    // glCullFace(GL_BACK);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT,  GL_NICEST);

    /* speedups */
    glEnable(GL_DITHER);
    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    /* light */
    GLfloat ambient[] = {0.8f, 0.8f, 0.8f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

    glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light0_color);
    // glLightfv(GL_LIGHT0, GL_AMBIENT,  light0_color);
    glLightfv(GL_LIGHT1, GL_POSITION, light1_pos);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  light1_color);
    glEnable(GL_LIGHT0);
    // glEnable(GL_LIGHT1);
    glEnable(GL_LIGHTING);


    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

// GLUquadricObj* mQuadric = NULL;


void GLWidget::paintGL() {
    glEnable(GL_DEPTH_TEST);
    // Clear
    // glClearColor( 0.15f, 0.15f, 0.15f, 0.0f );
    glClearColor( 1.0, 1.0, 1.0, 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Render
    glLoadIdentity();

    GLfloat m[4][4];
    glTranslatef( camera()->pos(0), camera()->pos(1), -camera()->pos(2) );
    // glTranslated( camera()->pos );


    camera()->build_rotmatrix( m );
    glMultMatrixf( &m[0][0] );


    glEnable(GL_LIGHTING);

    glColor3d(1.0, 0.0, 0.0);

    // // 3D Stuff
    // glutSolidSphere(1.0, 10, 10);

    // 2D Stuff
    enable2D();
    window()->sim()->render();
    disable2D();
}

void GLWidget::resizeGL( int w, int h ) {
    glViewport(0, 0, (GLint) w, (GLint) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0f, (GLfloat)w/h, 1.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GLWidget::enable2D() {
    int winX = GL_WINDOW_WIDTH;
    int winY = GL_WINDOW_HEIGHT;
    int PPU = 300; // Pixel per unit
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();		

    glDisable(GL_LIGHTING | GL_DEPTH_TEST);
    glDepthMask(0);
    // glOrtho(0, winX, winY,0,-1,1);
    glOrtho(-0.5 * winX / PPU, 0.5 * winX / PPU, -0.2 * winY / PPU, 0.8 * winY / PPU,-1,1);


    glViewport(0,0,winX,winY);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GLWidget::disable2D() {
    int winX = GL_WINDOW_WIDTH;
    int winY = GL_WINDOW_HEIGHT;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();		

    glEnable(GL_DEPTH_TEST | GL_LIGHTING);
    glDepthMask(1);
    gluPerspective(45.0f, (double)winX/(double)winY, 1.0, 100.0);

    glViewport(0,0,winX,winY);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


void GLWidget::mousePressEvent(QMouseEvent* event) {
    QPoint m1 = event->pos();
    camera()->begin = Eigen::Vector2d( m1.x(), m1.y() );
}

void GLWidget::mouseReleaseEvent(QMouseEvent* event) {
    QPoint m1 = event->pos();
}


void GLWidget::mouseMoveEvent(QMouseEvent* event) {
    QPoint m1 = event->pos();
    if ( (event->buttons() & Qt::LeftButton) != 0) {
        if ( (qApp->keyboardModifiers() & Qt::ShiftModifier) != 0 ) {
            float dx = m1.x() - camera()->begin.x();
            float dy = m1.y() - camera()->begin.y();
            camera()->pos(0) += 0.05 * dx;
            camera()->pos(1) -= 0.05 * dy;
        } else if ( (qApp->keyboardModifiers() & Qt::ControlModifier) != 0 ) {
            float amount = (m1.y() - camera()->begin.y())
                + (m1.x() - camera()->begin.x());
            camera()->pos(2) += 0.1 * amount;
        } else {
            float sx = size().width();
            float sy = size().height();
            camera()->add_quat(
                (2.0 * camera()->begin.x() - sx) / sx,
                (sy - 2.0 * camera()->begin.y()) / sy,
                (2.0 * m1.x() - sx) / sx,
                (sy - 2.0 * m1.y()) / sy
                );
        }
    }
    camera()->begin = Eigen::Vector2d( m1.x(), m1.y() );
}

} // namespace gui
} // namespace disneysimple
