/* DisneyLearning, Copyright (c) 2014, Disney Research
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
 * Disney Research Pittsburgh
 */

#include "SimBox2D.h"
#include "utils/CppCommon.h"
#include "utils/LoadOpengl.h"
#include "utils/Option.h"
#include <Box2D/Box2D.h>

namespace disney {
namespace simulation {

////////////////////////////////////////////////////////////
// struct Box2dsimulationImp;
struct SimBox2DImp {
    b2World* world;
    b2Body* ground;
    b2Body* board;
    b2Body* wheel;
    b2Body* l1;
    b2Body* r1;
    b2Body* l2;
    b2Body* r2;
    std::vector<b2Body*> bodies;
    
    SimBox2DImp();
    ~SimBox2DImp();

    void drawBody(b2Body* body);
    void drawShape(b2Fixture* fixture, const b2Transform& xf);
    Eigen::VectorXd toWorldPosList(const b2Contact* c);
    void drawContact(const b2Contact* c);
    void drawContact(const Eigen::Vector2d& c);
};

SimBox2DImp::SimBox2DImp() {
    const double SKIN = 0.001; // Default is 0.01;
    const double MU   = 1.0;
    const double WIDTH  = 0.005;

    // Define the gravity vector.
    b2Vec2 gravity(0.0f, -9.81f);
    // Construct a world object, which will hold and simulate the rigid bodies.
    world = new b2World(gravity);

    // Define the ground body.
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, -1.0f);
    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    ground = world->CreateBody(&groundBodyDef);
    // Define the ground box shape.
    b2PolygonShape groundBox;
    // The extents are the half-widths of the box.
    groundBox.SetAsBox(0.14f, 1.0f - SKIN);
    // Add the ground fixture to the ground body.
    ground->CreateFixture(&groundBox, 0.0f);



    // Wheel
    {
        // Define the dynamic body. We set its position and call the body factory.
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(0.0f, 0.06f);
        b2Body* body = world->CreateBody(&bodyDef);
        // Define another box shape for our dynamic body.
        b2CircleShape shape;
        shape.m_p.Set(0.0f, 0.0f);
        shape.m_radius = 0.05;
        // Define the dynamic body fixture.
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        // Set the box density to be non-zero, so it will be dynamic.
        fixtureDef.density = 200.0f;
        // Override the default friction.
        fixtureDef.friction = MU;
        // Add the shape to the body.
        body->CreateFixture(&fixtureDef);

        this->wheel = body;
    }

    // Board
    {
        // Define the dynamic body. We set its position and call the body factory.
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(0.0f, 0.12f);
        b2Body* body = world->CreateBody(&bodyDef);
        // Define another box shape for our dynamic body.
        b2PolygonShape shape;
        shape.SetAsBox(0.3f, WIDTH);
        shape.m_radius = SKIN; 

        // Define the dynamic body fixture.
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        // Set the box density to be non-zero, so it will be dynamic.
        fixtureDef.density = 2.0 / (4.0 * 0.3 * 0.005);
        // Override the default friction.
        fixtureDef.friction = MU;
        // Add the shape to the body.
        body->CreateFixture(&fixtureDef);
        this->board = body;
    }


    const double height = 0.5;
    for (int i = 0; i < 4; i++) {
        // Define the dynamic body. We set its position and call the body factory.
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        switch(i) {
        case 0: bodyDef.position.Set(-0.1f, 0.12 + 1.0 * height); break;
        case 1: bodyDef.position.Set( 0.1f, 0.12 + 1.0 * height); break;
        case 2:
            bodyDef.position.Set(-0.05f, 0.12 + 2.0 * height);
            bodyDef.angle = PI / 2.0;
            break;
        case 3:
            bodyDef.position.Set(0.05f, 0.12 + 2.0 * height);
            bodyDef.angle = -PI / 2.0;
            break;
        }
        b2Body* body = world->CreateBody(&bodyDef);
        // Define another box shape for our dynamic body.
        b2PolygonShape shape;
        double sx;
        double sy;
        switch(i) {
        case 0: 
        case 1:
            sx = WIDTH; sy = height; break;
            // shape.SetAsBox(width, height); break;
        case 2:
        case 3: 
            // shape.SetAsBox(0.05, width); break;
            // shape.SetAsBox(width, 0.05); break;
            sx = WIDTH; sy = 0.05; break;
        }
        shape.SetAsBox(sx, sy);
        shape.m_radius = SKIN; 
        // Define the dynamic body fixture.
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        // Set the box density to be non-zero, so it will be dynamic.
        fixtureDef.density = 15.0 / (4.0 * sx * sy);
        // Override the default friction.
        fixtureDef.friction = MU;
        // Add the shape to the body.
        body->CreateFixture(&fixtureDef);
        switch(i) {
        case 0: this->l1 = body; break;
        case 1: this->r1 = body; break;
        case 2: this->l2 = body; break;
        case 3: this->r2 = body; break;
        }
    }

    {
        b2WeldJointDef jointDef;
        jointDef.Initialize(l2, r2, 0.5 * (l2->GetPosition() + r2->GetPosition()));
        b2WeldJoint* joint = (b2WeldJoint*)world->CreateJoint(&jointDef);
    }
    {
        b2RevoluteJointDef jointDef;
        b2Vec2 anchor;
        anchor.x = l1->GetPosition().x;
        anchor.y = l2->GetPosition().y;
        jointDef.Initialize(l1, l2, anchor);
        b2RevoluteJoint* joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
    }
    {
        b2RevoluteJointDef jointDef;
        b2Vec2 anchor;
        anchor.x = r1->GetPosition().x;
        anchor.y = r2->GetPosition().y;
        jointDef.Initialize(r1, r2, anchor);
        b2RevoluteJoint* joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
    }
    {
        b2RevoluteJointDef jointDef;
        b2Vec2 anchor;
        anchor.x = l1->GetPosition().x;
        anchor.y = board->GetPosition().y;
        jointDef.Initialize(l1, board, anchor);
        b2RevoluteJoint* joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
    }
    {
        b2RevoluteJointDef jointDef;
        b2Vec2 anchor;
        anchor.x = r1->GetPosition().x;
        anchor.y = board->GetPosition().y;
        jointDef.Initialize(r1, board, anchor);
        b2RevoluteJoint* joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
    }


    bodies.push_back(wheel);
    bodies.push_back(board);
    bodies.push_back(l1);
    bodies.push_back(r1);
    bodies.push_back(l2);
    bodies.push_back(r2);

    for (int i = 0; i < bodies.size(); i++) {
        LOG(INFO) << i << " : " << bodies[i]->GetMass() << " " << bodies[i]->GetInertia();
    }
}

SimBox2DImp::~SimBox2DImp() {
}

void SimBox2DImp::drawBody(b2Body* body) {
    const b2Transform& xf = body->GetTransform();
    for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
        drawShape(f, xf);
    }
}

void SimBox2DImp::drawShape(b2Fixture* fixture, const b2Transform& xf) {
    if (fixture->GetType() == b2Shape::e_polygon) {
        b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
        int32 n = poly->m_count;

        glBegin(GL_POLYGON);
        for (int i = 0; i < n; i++) {
            b2Vec2 v = b2Mul(xf, poly->m_vertices[i]);
            
            glVertex2d(v.x, v.y);
        }
        glEnd();
    } else if (fixture->GetType() == b2Shape::e_circle) {
        b2CircleShape* circle = (b2CircleShape*)fixture->GetShape();

        b2Vec2  c = b2Mul(xf, circle->m_p);
        float32 r = circle->m_radius;
        glBegin(GL_LINE_LOOP);

        glVertex2d(c.x, c.y);
        for (double th = 0.0; th <= 2 * PI + 0.0001; th += (PI / 10.0) ) {
            b2Vec2 v = b2Mul(xf, b2Vec2(r * cos(th), r * sin(th)));
            glVertex2d(v.x, v.y);
        }
        glEnd();
    }


}
Eigen::VectorXd SimBox2DImp::toWorldPosList(const b2Contact* c) {
    const b2Manifold* m = c->GetManifold();
    Eigen::VectorXd ret(m->pointCount * 2);

    b2WorldManifold wm;
    c->GetWorldManifold(&wm);
    int ptr = 0;
    for (int i = 0; i < m->pointCount; i++) {
        b2Vec2 p = wm.points[i];
        ret(ptr++) = p.x;
        ret(ptr++) = p.y;
    }
    return ret;
}

void SimBox2DImp::drawContact(const b2Contact* c) {
    const b2Manifold* m = c->GetManifold();
    b2WorldManifold wm;
    c->GetWorldManifold(&wm);
    for (int i = 0; i < m->pointCount; i++) {
        glColor4f(1.0f, 0.0f, 1.0f, 1.0f);
        glPushMatrix();
        b2Vec2 p = wm.points[i];
        glTranslatef(p.x, p.y, 0.0);

        glBegin(GL_TRIANGLE_FAN);
        const float r = 0.01;
        for (unsigned int a = 0; a < 64; ++a) {
            float ang = (a / 64.0f) * 2.0f * float(M_PI);
            glVertex2f(cosf(ang) * r, sinf(ang) * r);
        }
        glEnd();
        glPopMatrix();
    }
}

void SimBox2DImp::drawContact(const Eigen::Vector2d& c) {
    glColor4f(1.0f, 0.0f, 1.0f, 1.0f);
    glPushMatrix();
    b2Vec2 p(c(0), c(1));
    glTranslatef(p.x, p.y, 0.0);

    glBegin(GL_TRIANGLE_FAN);
    const float r = 0.01;
    for (unsigned int a = 0; a < 64; ++a) {
        float ang = (a / 64.0f) * 2.0f * float(M_PI);
        glVertex2f(cosf(ang) * r, sinf(ang) * r);
    }
    glEnd();
    glPopMatrix();
}


//
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
// class SimBox2D implementation
SimBox2D::SimBox2D()
    : Simulator(SIMTYPE_BOX2D)
    , imp(NULL)
{
}

SimBox2D::~SimBox2D() {
    if (imp) { delete imp; imp = NULL; }
}

Simulator* SimBox2D::init() {
    imp = new SimBox2DImp;

    int n = 6;
    int m = 2 * n;
    Eigen::VectorXd xEq(m);
    xEq << 0, 0, 0, 0, PI_2, -PI_2
         , 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd xOffset(m);
    double angIni = utils::Option::read("simulation.init.angle").toDouble();
    xOffset << 0.0, (angIni * PI / 180), 0, 0, 0, 0
        , 0, 0, 0, 0, 0, 0;
    // xOffset << 0.2, 0.3, 0.2, 0.2, -0.2, -0.2
    //     , 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd cState = xEq + xOffset;
    setState(cState);

    mTorque = Eigen::VectorXd::Zero( numDimTorque() );

    clearHistory();
    LOG(INFO) << FUNCTION_NAME() << " OK";
    return this;
}

// Full State functions -- for maximal simulators. In default, state == full state
void SimBox2D::setFullState(const Eigen::VectorXd& _fullState) {
    const Eigen::VectorXd& state = _fullState;
    int ptr = 0;
    for (int i = 0; i < imp->bodies.size(); i++) {
        b2Body* body = imp->bodies[i];

        double px = state(ptr++);
        double py = state(ptr++);
        double a  = state(ptr++);
        double vx = state(ptr++);
        double vy = state(ptr++);
        double w  = state(ptr++);

        body->SetTransform( b2Vec2(px, py), a);
        body->SetLinearVelocity( b2Vec2(vx, vy) );
        body->SetAngularVelocity( w );
    }        
}

Eigen::VectorXd SimBox2D::fullState() const {
    Eigen::VectorXd state(6 * 6);
    int ptr = 0;
    for (int i = 0; i < imp->bodies.size(); i++) {
        b2Body* body = imp->bodies[i];

        b2Vec2  p = body->GetPosition();
        float32 a = body->GetAngle();
        b2Vec2  v = body->GetLinearVelocity();
        float32 w = body->GetAngularVelocity();
        state(ptr++) = p.x;
        state(ptr++) = p.y;
        state(ptr++) = a;
        state(ptr++) = v.x;
        state(ptr++) = v.y;
        state(ptr++) = w;
    }
    return state;
}


void SimBox2D::integrate() {
    // Apply Torque first
    applyTorque();
    
    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float32 timeStep = mTimestep;
    // int32 velocityIterations = 6;
    // int32 positionIterations = 2;
    int32 velocityIterations = 750;
    int32 positionIterations = 20;

    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    imp->world->Step(timeStep, velocityIterations, positionIterations);
    // imp->world->Step(timeStep, velocityIterations, positionIterations);

    std::vector<Eigen::Vector2d> contacts;
    for (b2Contact* c = imp->world->GetContactList(); c; c = c->GetNext()) { 
        Eigen::VectorXd p_list = imp->toWorldPosList(c);
        for (int i = 0; i < p_list.size(); i += 2) {
            Eigen::Vector2d p = p_list.segment(i, 2);
            contacts.push_back(p);
        }
    }
    mContacts = Eigen::VectorXd::Zero( contacts.size() * 2 );
    for (int i = 0; i < contacts.size(); i++) {
        mContacts(2 * i + 0) = contacts[i](0);
        mContacts(2 * i + 1) = contacts[i](1);
    }
}

void SimBox2D::applyTorque() {
    // LOG(INFO) << "box2d.x = " << state().transpose();
    // LOG(INFO) << "box2d.u = " << mTorque.transpose();

    const Eigen::VectorXd& u = mTorque;

    for (int i = 2, j = 0; i < imp->bodies.size(); i++, j++) {
        b2Body* body = imp->bodies[i];
        b2Body* parent = NULL;
        switch(i) {
        case 2: parent = imp->board; break;
        case 3: parent = imp->board; break;
        case 4: parent = imp->l1; break;
        case 5: parent = imp->r1; break;
        }
        double tau = 0.5 * u(j);
        body->ApplyTorque(tau, true);
        parent->ApplyTorque(-tau, true);
    }    
}

void SimBox2D::reset() {
    Simulator::reset();
    for (int i = 0; i < imp->bodies.size(); i++) {
        b2Body* body = imp->bodies[i];
        body->SetAwake(true);
    }    
}

void SimBox2D::render() {
    glPushMatrix();
    // {
    //     double scale = 1.0;
    //     glTranslated(-1.0, 0.0, 0.0);
    //     glScaled(scale, scale, scale);
    // }
    glColor3d(0.0, 0.3, 0.5);
    imp->drawBody(imp->ground);
    glColor3d(1.0, 0.0, 0.0);
    imp->drawBody(imp->wheel);
    glColor3d(0.0, 0.0, 0.0);
    imp->drawBody(imp->board);
    glColor3d(0.0, 1.0, 0.0);
    imp->drawBody(imp->l1);
    imp->drawBody(imp->l2);
    glColor3d(0.0, 0.0, 1.0);
    imp->drawBody(imp->r1);
    imp->drawBody(imp->r2);
    glPopMatrix();

    for (int i = 0; i < mContacts.size(); i += 2) {
        Eigen::Vector2d c = mContacts.segment<2>(i);
        imp->drawContact( c );
    }
    // for (b2Contact* c = imp->world->GetContactList(); c; c = c->GetNext()) { 
    //     imp->drawContact(c);
    // }

}

Eigen::VectorXd SimBox2D::state() const {
    Eigen::VectorXd ret( numDimState() );

    for (int i = 0; i < imp->bodies.size(); i++) {
        b2Body* body = imp->bodies[i];
        float32 a = body->GetAngle();
        float32 w = body->GetAngularVelocity();

        b2Body* parent = NULL;
        float pa = 0.0;
        float pw = 0.0;
        switch(i) {
        case 1: parent = imp->wheel; break;
        case 2: parent = imp->board; break;
        case 3: parent = imp->board; break;
        case 4: parent = imp->l1; break;
        case 5: parent = imp->r1; break;
        }

        if (parent) {
            pa = parent->GetAngle();
            pw = parent->GetAngularVelocity();
        }
        
        ret(i + 0) = a - pa;
        ret(i + 6) = w - pw;
    }
    return ret;
}


void SimBox2D::setState(const Eigen::VectorXd& _state) {
    const double SKIN = 0.0001;
    // Parameters
    double offset = 0;
    double radius = 0.05;
    double rw     = radius;
    double aXmin  = -0.6;
    double aXmax  = 0.6;
    double aYmin  = 2*rw-0.3;
    double aYmax  = 2*rw+1.5;
    double lb     = 0.6;

    double lrl1 = 1;
    double lrl2 = 0.1;
    double lll1 = 1;
    double lll2 = 0.1;

    int n = 6;
    Eigen::VectorXd q = _state.head(n);
    double alphaw  = q(0);
    double alphab  = q(1);
    double al      = 0.0;
    double ar      = 0.0;
    double thetal1 = q(2);
    double thetar1 = q(3);
    double thetal2 = q(4);
    double thetar2 = q(5);

    const int X = 1;
    const int Y = 2;
    // Set wheel
    Eigen::Vector3d wheel;
    wheel << 0,
        offset-alphaw*rw,
        rw + SKIN;
    imp->wheel->SetTransform(b2Vec2(wheel(X), wheel(Y)), alphaw);

    // Set board
    Eigen::Vector3d boardRight;
    boardRight << 0,
        offset- alphaw*rw - cos(alphab + alphaw)*(lb/2 - alphab*rw) - rw*sin(alphab + alphaw),
        rw - sin(alphab + alphaw)*(lb/2 - alphab*rw) + rw*cos(alphab + alphaw);

    Eigen::Vector3d boardLeft;
    boardLeft << 0,
        offset+cos(alphab + alphaw)*(lb/2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw),
        rw + sin(alphab + alphaw)*(lb/2 + alphab*rw) + rw*cos(alphab + alphaw);     

    Eigen::Vector3d board = 0.5 * (boardLeft + boardRight);
    board(Y) += SKIN * 2;
    imp->board->SetTransform(b2Vec2(board(X), board(Y)), alphab + imp->wheel->GetAngle());

    // Set right links
    Eigen::Vector3d rightCart;
    rightCart << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw),
        rw + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw)  ;          
    Eigen::Vector3d rightLink1;
    rightLink1 << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - rw*sin(alphab + alphaw) - lll1*sin(alphab + alphaw + thetal1),
        rw + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw) + lll1*cos(alphab + alphaw + thetal1)  ;
    Eigen::Vector3d rightLink2;
    rightLink2 << 0,
        offset+cos(alphab + alphaw)*(al + lll2 + alphab*rw) - alphaw*rw - lll2*sin(alphab + alphaw + thetal1 + thetal2) - rw*sin(alphab + alphaw) - lll1*sin(alphab + alphaw + thetal1),
        rw + lll2*cos(alphab + alphaw + thetal1 + thetal2) + sin(alphab + alphaw)*(al + lll2 + alphab*rw) + rw*cos(alphab + alphaw) + lll1*cos(alphab + alphaw + thetal1)  ;


    Eigen::Vector3d r1 = 0.5 * (rightCart + rightLink1);
    imp->r1->SetTransform(b2Vec2(r1(X), r1(Y)), thetar1 + imp->board->GetAngle());
    Eigen::Vector3d r2 = 0.5 * (rightLink1 + rightLink2);
    imp->r2->SetTransform(b2Vec2(r2(X), r2(Y)), thetar2 + imp->r1->GetAngle());

    // Set left links
    Eigen::Vector3d leftCart;
    leftCart << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - rw*sin(alphab + alphaw) - alphaw*rw,
        rw + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw);
    Eigen::Vector3d leftLink1;
    leftLink1 << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - rw*sin(alphab + alphaw) - alphaw*rw - lrl1*sin(alphab + alphaw + thetar1),
        rw + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw) + lrl1*cos(alphab + alphaw + thetar1)   ;
    Eigen::Vector3d leftLink2;
    leftLink2 << 0,
        offset+cos(alphab + alphaw)*(ar - lrl2 + alphab*rw) - lrl2*sin(alphab + alphaw + thetar1 + thetar2) - rw*sin(alphab + alphaw) - alphaw*rw - lrl1*sin(alphab + alphaw + thetar1),
        rw + lrl2*cos(alphab + alphaw + thetar1 + thetar2) + rw*cos(alphab + alphaw) + sin(alphab + alphaw)*(ar - lrl2 + alphab*rw) + lrl1*cos(alphab + alphaw + thetar1)  ;

    Eigen::Vector3d l1 = 0.5 * (leftCart + leftLink1);
    imp->l1->SetTransform(b2Vec2(l1(X), l1(Y)), thetal1 + imp->board->GetAngle());
    Eigen::Vector3d l2 = 0.5 * (leftLink1 + leftLink2);
    imp->l2->SetTransform(b2Vec2(l2(X), l2(Y)), thetal2 + imp->l1->GetAngle());

    LOG(INFO) << FUNCTION_NAME() << " OK";
    
}


// class SimBox2D ends
////////////////////////////////////////////////////////////



} // namespace simulation
} // namespace disney


