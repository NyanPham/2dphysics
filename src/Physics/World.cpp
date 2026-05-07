#include "World.h"
#include "Constants.h"
#include "CollisionDetection.h"
#include <iostream>

World::World(float gravity) {
    G = -gravity;
    std::cout << "World constructor called!" << std::endl;
}

World::~World() {
    for (auto body: bodies) {
        delete body; 
    }
    std::cout << "World destructor called!" << std::endl;
}

void World::AddBody(Body* body) {
    bodies.push_back(body);
}

std::vector<Body*>& World::GetBodies() {
    return bodies;
}

void World::AddConstraint(Constraint* constraint) {
    constraints.push_back(constraint);
}

std::vector<Constraint*>& World::GetConstraints() {
    return constraints;
}

void World::AddForce(const Vec2& force) {
    forces.push_back(force);
}

void World::AddTorque(float torque) {
    torques.push_back(torque);
}

void World::Update(float dt) {
    // loop all bodies of the world applying forces 
    for (auto body: bodies) {
        // apply the weight force to all bodies 
        Vec2 weight = Vec2(0.0, body->mass * G * PIXELS_PER_METER);
        body->AddForce(weight);
   
        // apply forces to all bodies 
        for (auto force: forces)
            body->AddForce(force);

        // apply torque to all bodies 
        for (auto torque: torques) 
            body->AddTorque(torque);
    }
    
    // integrate all the forces 
    for (auto body: bodies) {
        body->IntegrateForces(dt);
    }

    // solve all constraints
    for (auto& constraint: constraints) {
        constraint->Solve();
    }

    for (auto body: bodies) {
        body->IntegrateVelocities(dt);
    }

    // collision detection and resolution for all bodies of the world 
    CheckCollisions();
}

void World::CheckCollisions() {
    // check all the bodies with all other bodies detecting collisions 
    for (size_t i = 0; i <= bodies.size() - 1; i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            Body* a = bodies[i];
            Body* b = bodies[j]; 
            a->isColliding = false;
            b->isColliding = false;

            Contact contact;
            if (CollisionDetection::IsColliding(a, b, contact)) {
                // resolve the collision 
                contact.ResolveCollision();
            }
        }
    }
}
