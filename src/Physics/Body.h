#ifndef BODY_H
#define BODY_H

#include "Vec2.h"
#include "Shape.h"

struct Body {
    bool isColliding = false;

    // Linear motion
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;
    
    // Angular motion
    float rotation;
    float angularVelocity;
    float angularAcceleration;

    // forces and torque
    Vec2 sumForces;
    float sumTorque;

    // mass and moment of inertia
    float mass;
    float invMass;
    float I;
    float invI;
    
    // pointer to the shape/geometry of this rigid body 
    Shape* shape = nullptr;

    Body(const Shape& shape, float x, float y, float mass); 
    ~Body();

    bool IsStatic() const;
        
    void AddForce(const Vec2& force);
    void AddTorque(float torque);
    void ClearForces(void);
    void ClearTorque(void);

    void IntegrateLinear(float dt);
    void IntegrateAngular(float dt);

    void Update(float dt);
};

#endif
