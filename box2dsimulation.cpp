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
    b2Body* groundBody;
    b2Body* board;
    b2Body* wheel;
    b2Body* l1;
    b2Body* l2;
    b2Body* r1;
    b2Body* r2;
    
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
    groundBodyDef.position.Set(0.0f, -10.0f);
    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    groundBody = world->CreateBody(&groundBodyDef);
    // Define the ground box shape.
    b2PolygonShape groundBox;
    // The extents are the half-widths of the box.
    groundBox.SetAsBox(50.0f, 10.0f);
    // Add the ground fixture to the ground body.
    groundBody->CreateFixture(&groundBox, 0.0f);

    {
        // Define the dynamic body. We set its position and call the body factory.
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(0.0f, 0.12f);
        b2Body* body = world->CreateBody(&bodyDef);
        // Define another box shape for our dynamic body.
        b2PolygonShape shape;
        shape.SetAsBox(0.3f, 0.005f);
        // Define the dynamic body fixture.
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        // Set the box density to be non-zero, so it will be dynamic.
        fixtureDef.density = 1.0f;
        // Override the default friction.
        fixtureDef.friction = 0.3f;
        // Add the shape to the body.
        body->CreateFixture(&fixtureDef);
        this->board = body;
    }

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
        fixtureDef.density = 1.0f;
        // Override the default friction.
        fixtureDef.friction = 0.3f;
        // Add the shape to the body.
        body->CreateFixture(&fixtureDef);

        this->wheel = body;
    }

    const double width  = 0.005;
    const double height = 0.5;
    for (int i = 0; i < 4; i++) {
        // Define the dynamic body. We set its position and call the body factory.
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        switch(i) {
        case 0: bodyDef.position.Set(-0.1f, 0.12 + 1.0 * height); break;
        case 1: bodyDef.position.Set(-0.05f, 0.12 + 2.0 * height); break;
        case 2: bodyDef.position.Set( 0.1f, 0.12 + 1.0 * height); break;
        case 3: bodyDef.position.Set(0.05f, 0.12 + 2.0 * height); break;
        }
        b2Body* body = world->CreateBody(&bodyDef);
        // Define another box shape for our dynamic body.
        b2PolygonShape shape;
        switch(i) {
        case 0: 
        case 2:
            shape.SetAsBox(width, height); break;
        case 1: 
        case 3: 
            shape.SetAsBox(0.05, width); break;
        }
        // Define the dynamic body fixture.
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        // Set the box density to be non-zero, so it will be dynamic.
        fixtureDef.density = 1.0f;
        // Override the default friction.
        fixtureDef.friction = 0.3f;
        // Add the shape to the body.
        body->CreateFixture(&fixtureDef);
        switch(i) {
        case 0: this->l1 = body; break;
        case 1: this->l2 = body; break;
        case 2: this->r1 = body; break;
        case 3: this->r2 = body; break;
        }
        
    }

    {
        b2RevoluteJointDef jointDef;
        jointDef.Initialize(l2, r2, 0.5 * (l2->GetPosition() + r2->GetPosition()));
        b2RevoluteJoint* joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
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
}

void Box2dSimulation::step() {
    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float32 timeStep = 1.0f / 100.0f;
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

}

void Box2dSimulation::render() {
    {
        double scale = 1.0;
        glTranslated(-1.0, 0.0, 0.0);
        glScaled(scale, scale, scale);
    }
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
}

} // namespace sim
} // namespace disneysimple



