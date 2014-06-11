#include "box2dsimulation.h"
#include "cppcommon.h"
#include "LoadOpengl.h"
#include <Box2D/Box2D.h>

namespace disneysimple {
namespace sim {

////////////////////////////////////////////////////////////
// struct Box2dsimulationImp;
struct Box2dSimulationImp {
    b2World* world;
    b2Body* ground;
    b2Body* board;
    b2Body* wheel;
    b2Body* l1;
    b2Body* r1;
    b2Body* l2;
    b2Body* r2;
    std::vector<b2Body*> bodies;
    
    Box2dSimulationImp();
    ~Box2dSimulationImp();

    void drawBody(b2Body* body);
    void drawShape(b2Fixture* fixture, const b2Transform& xf);
};

Box2dSimulationImp::Box2dSimulationImp() {
    // Define the gravity vector.
    b2Vec2 gravity(0.0f, -10.0f);
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
    groundBox.SetAsBox(1.0f, 1.0f);
    // Add the ground fixture to the ground body.
    ground->CreateFixture(&groundBox, 0.0f);


    const double SKIN = 0.0001; // Default is 0.01;
    const double MU   = 10.0;

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
        shape.SetAsBox(0.3f, 0.005f);
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


    const double width  = 0.005;
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
            sx = width; sy = height; break;
            // shape.SetAsBox(width, height); break;
        case 2:
        case 3: 
            // shape.SetAsBox(0.05, width); break;
            // shape.SetAsBox(width, 0.05); break;
            sx = width; sy = 0.05; break;
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

Box2dSimulationImp::~Box2dSimulationImp() {
}

void Box2dSimulationImp::drawBody(b2Body* body) {
    const b2Transform& xf = body->GetTransform();
    for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
        drawShape(f, xf);
    }
}

void Box2dSimulationImp::drawShape(b2Fixture* fixture, const b2Transform& xf) {
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
        glBegin(GL_POLYGON);
        for (double th = 0.0; th < 2 * PI; th += (PI / 10.0) ) {
            double x = c.x + r * cos(th);
            double y = c.y + r * sin(th);
            glVertex2d(x, y);
        }
        glEnd();
    }


}
//
////////////////////////////////////////////////////////////


Box2dSimulation::Box2dSimulation() {
    init();

}

Box2dSimulation::~Box2dSimulation() {
}

void Box2dSimulation::init() {
    imp = new Box2dSimulationImp;

    int n = 6;
    int m = 2 * n;
    Eigen::VectorXd xEq(m);
    xEq << 0, 0, 0, 0, PI_2, -PI_2
         , 0, 0, 0, 0, 0, 0;
    // xEq << 0, 0, 0, 0, 0, 0
    //      , 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd xOffset(m);
    double angIni = 2;
    xOffset << 0.0, (angIni * PI / 180), 0, 0, 0, 0
        , 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd cState = xEq + xOffset;
    setControlState(cState);

    mStateHistory.push_back( getState() );
}

void Box2dSimulation::control() {
    const int n = 6;

    // Parameters
    double radius = 0.05;
    double rw     = radius;
    double lrl1 = 1;
    double lrl2 = 0.1;
    double lll1 = 1;
    double lll2 = 0.1;
    double maxTorq = 100; 


    // Feedback Matrix
    Eigen::MatrixXd C(5, 2 * n);
    C << - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, - lll1 - lll2 - 2*rw, - lll1 - lll2, - lll1 - lll2, 0, -lll2, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,  
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,    
        1, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0;  


    Eigen::MatrixXd F(4, 5);
    F << -2716691.61124073, -1189377.26019503, 953603.332318603, 10071.8805576070, 768507.689768547,
        -2716691.61123261, -1189377.26019358, 953603.332315551, 10071.8805576529, 768507.689765388,
        2716691.61123813, 1189377.26019453, -953603.332317613, -10071.8805576191, -768507.689767548,
        2716691.61124318, 1189377.26019541, -953603.332319511, -10071.8805575885, -768507.689769501;

    // F *= 0.01;

    Eigen::MatrixXd K = F * C;

    // State 
    Eigen::VectorXd x = getControlState();
    
    // Equilibrium state
    Eigen::VectorXd qEq(n);
    qEq << 0.0, 0.0, 0.0, 0.0, PI/2.0, -PI/2.0;
    Eigen::VectorXd dqEq(n);
    dqEq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd xEq(n * 2);
    xEq.head(n) = qEq;
    xEq.tail(n) = dqEq;
        
    // Calculate u
    Eigen::VectorXd u = -K * (x - xEq);
    for(int i = 0; i < u.size(); i++) {
        if (fabs(u(i)) > maxTorq) {
            if (u(i) > 0) {
                u(i) = maxTorq;
            } else {
                u(i) = -maxTorq;
            }
        }
    }

    LOG(INFO) << "box2d.u = " << u.transpose();
    
    for (int i = 2, j = 0; i < imp->bodies.size(); i++, j++) {
        b2Body* body = imp->bodies[i];
        b2Body* parent = NULL;
        switch(i) {
        case 2: parent = imp->board; break;
        case 3: parent = imp->board; break;
        case 4: parent = imp->l1; break;
        case 5: parent = imp->r1; break;
        }
        double tau = -0.5 * u(j);
        body->ApplyTorque(tau, true);
        parent->ApplyTorque(-tau, true);
    }    
    
}

void Box2dSimulation::step() {
    if ( (mStateHistory.size()) % 2 == 1) {
        control();
    }

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float32 timeStep = 1.0f / 1000.0f;
    int32 velocityIterations = 6;
    int32 positionIterations = 2;

    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    imp->world->Step(timeStep, velocityIterations, positionIterations);

    // // Now print the position and angle of the body.
    // b2Vec2 position = imp->body->GetPosition();
    // float32 angle = imp->body->GetAngle();

    // // printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
    // LOG(INFO) << position.x << ", " <<  position.y << ", " <<  angle;

    mStateHistory.push_back( getState() );

    // LOG_EVERY_N(INFO, 10) << getControlState().head(6).transpose();

}

void Box2dSimulation::render() {
    glPushMatrix();
    {
        double scale = 1.0;
        glTranslated(-1.0, 0.0, 0.0);
        glScaled(scale, scale, scale);
    }
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
}

Eigen::VectorXd Box2dSimulation::getState() {
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

void Box2dSimulation::setState(const Eigen::VectorXd& state) {
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

void Box2dSimulation::reset() {
    updateToHistory(0);
    mStateHistory.clear();
    mStateHistory.push_back( getState() );
}

void Box2dSimulation::updateToHistory(int index) {
    Eigen::VectorXd state = mStateHistory[index];
    setState(state);
}

Eigen::VectorXd Box2dSimulation::getControlState() {
    Eigen::VectorXd state(6 * 2);

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
        
        state(i + 0) = a - pa;
        state(i + 6) = w - pw;

        
    }
    return state;
}

void Box2dSimulation::setControlState(const Eigen::VectorXd& state) {
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
    Eigen::VectorXd q = state.head(n);
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
        rw;
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
    imp->board->SetTransform(b2Vec2(board(X), board(Y)), alphab - imp->wheel->GetAngle());

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

    
}

} // namespace sim
} // namespace disneysimple



