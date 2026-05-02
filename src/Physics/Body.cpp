#include "Body.h"
#include <cmath>
#include <iostream>

Body::Body(const Shape& shape, float x, float y, float mass) {
    this->shape = shape.Clone();
    this->position = Vec2(x, y);
    this->velocity = Vec2(0, 0);
    this->acceleration = Vec2(0, 0);
    this->rotation = 0.0;
    this->angularVelocity = 0.0;
    this->angularAcceleration = 0.0;
    this->sumForces = Vec2(0, 0);
    this->sumTorque = 0.0;
    this->restitution = 1.0;
    this->friction = 0.7;
    this->mass = mass;
    if (mass != 0.0) {
        this->invMass = 1.0 / mass;
    } else {
        this->invMass = 0.0;
    }
    I = shape.GetMomentOfInertia() * mass;
    if (I != 0.0) {
        this->invI = 1.0 / I;
    } else {
        this->invI = 0.0;
    }
    std::cout << "Body constructor called!" << std::endl;
}

Body::~Body() {
    delete shape;
    std::cout << "Body destructor called!" << std::endl;
}

bool Body::IsStatic() const {
    const float epsilon = 0.005f;
    return fabs(invMass - 0.0) < epsilon;
}

void Body::AddForce(const Vec2& force) {
    sumForces += force;
}

void Body::AddTorque(float torque) {
    sumTorque += torque;
}

void Body::ClearForces(void) {
    sumForces = Vec2(0.0, 0.0);
}

void Body::ClearTorque(void) {
    sumTorque = 0.0;
}

void Body::ApplyImpulse(const Vec2& j) {
    if (IsStatic()) {
        return;
    }

    velocity += j * invMass;
}

void Body::ApplyImpulse(const Vec2& j, const Vec2& r) {
    if (IsStatic()) {
        return;
    } 
    velocity += j * invMass;
    angularVelocity += r.Cross(j) * invI;
}

void Body::IntegrateLinear(float dt) {
    if (IsStatic()) {
        return;
    }

    // find the acceleration based on the forces that are being applied and the mass
    acceleration = sumForces * invMass;

    // integrate the acceleration to find the new velocity 
    velocity += acceleration * dt;

    // integrate the velocity to find the position
    position += velocity * dt;

    ClearForces();
}

void Body::IntegrateAngular(float dt) {
    if (IsStatic()) {
        return;
    }

    // find the acceleration based on the torque that are being applied and the moment of inertia
    angularAcceleration = sumTorque * invI;

    // integrate the angular acceleration to find the new angular velocity 
    angularVelocity += angularAcceleration * dt;

    // integrate the angular velocity to find the rotation angle
    rotation += angularVelocity * dt;

    ClearTorque();
}

void Body::Update(float dt) {
    IntegrateLinear(dt);
    IntegrateAngular(dt);
    bool isPolygon = shape->GetType() == POLYGON || shape->GetType() == BOX;
    if (isPolygon) {
        PolygonShape* polygonShape = (PolygonShape*) shape;
        polygonShape->UpdateVertices(rotation, position);
    }
}
