#include "World.h"
#include "Constants.h"
#include "CollisionDetection.h"
#include "Constraint.h"
#include "../Graphics.h"
#include <iostream>

World::World(float gravity) {
    G = -gravity;
    std::cout << "World constructor called!" << std::endl;
}

World::~World() {
    for (auto body: bodies) {
        delete body; 
    }
    for (auto constraint: constraints) {
        delete constraint;
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
    std::vector<PenetrationConstraint> penetrations;    

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

    // check all the bodies with all other bodies detecting collisions 
    for (size_t i = 0; i <= bodies.size() - 1; i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            Body* a = bodies[i];
            Body* b = bodies[j]; 
            
            std::vector<Contact> contacts;
            if (CollisionDetection::IsColliding(a, b, contacts)) {
                for (auto contact: contacts) {
                    Graphics::DrawCircle(contact.start.x, contact.start.y, 5, 0.0, 0xFF00FFFF);
                    Graphics::DrawCircle(contact.end.x, contact.end.y, 2, 0.0, 0xFF00FFFF);

                    // create a new penetration constraint 
                    PenetrationConstraint penetration(contact.a, contact.b, contact.start, contact.end, contact.normal);
                    penetrations.push_back(penetration);
                }
            }
        }
    }
    
    // solve all constraints
    /*
    for (auto& constraint: constraints) {
        constraint->PreSolve(dt);
    }
    for (auto& constraint: penetrations) {
        constraint.PreSolve(dt);
    }

    for (int i = 0; i < 5; i++) {
        for (auto& constraint: constraints) {
            constraint->Solve();
        }
        for (auto& constraint: penetrations) {
            constraint.Solve();
        }
    }
    for (auto& constraint: constraints) {
        constraint->PostSolve();
    }
    for (auto& constraint: penetrations) {
        constraint.PostSolve();
    }
    */
    for (auto body: bodies) {
        body->IntegrateVelocities(dt);
    }
}
