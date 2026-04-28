#include "./Force.h"
#include <algorithm>

Vec2 Force::GenerateDragForce(const Body& body, float k) {
    Vec2 dragForce = Vec2(0, 0);

    if (body.velocity.MagnitudeSquared() > 0) {
        // calculate the drag direction (inverse of the velocity unit vector)
        Vec2 dragDirection = body.velocity.UnitVector() * -1.0;

        // calculate the drag magnitude, k * |v|^2 
        float dragMagnitude = k * body.velocity.MagnitudeSquared();

        // generate the final drag force with direction and magnitude
        dragForce = dragDirection * dragMagnitude;
    }
    return dragForce;
}

Vec2 Force::GenerateFrictionForce(const Body& body, float k) {
    Vec2 frictionForce = Vec2(0, 0);

    // calculate the friction direction (inverse of the velocity unit vector)
    Vec2 frictionDirection = body.velocity.UnitVector() * -1.0;

    // calculate the friction magnitude
    float frictionMagnitude = k;

    // calculate the final friction force 
    frictionForce = frictionDirection * frictionMagnitude;

    return frictionForce;
}

Vec2 Force::GenerateGravitationalForce(const Body& a, const Body& b, float G, float minDistance, float maxDistance) {
    // calculate the distance between the two objects
    Vec2 d = (b.position - a.position);

    float distanceSquared = d.MagnitudeSquared();

    // clamp the values of the distance to allow for some interesting visual effects 
    distanceSquared = std::clamp(distanceSquared, minDistance, maxDistance);

    // calculate the direction of the attraction force 
    Vec2 attractionDirection = d.UnitVector();

    // calculate the strenght of the attraction force
    float attractionMagnitude = G * (a.mass * b.mass) / distanceSquared;

    // calculate the final resulting attraction force vector
    Vec2 attractionForce = attractionDirection * attractionMagnitude;

    return attractionForce;
}

Vec2 Force::GenerateSpringForce(const Body& body, Vec2 anchor, float restLength, float k) {
    // calculate the distance between the anchor and the object 
    Vec2 d = body.position - anchor;
    
    // find the spring displacement considering the rest length 
    float displacement = d.Magnitude() - restLength;

    // calculate the direction and the magitude of the spring force 
    Vec2 springDirection = d.UnitVector();
    float springMagnitude = -k * displacement;

    // calculate the final resulting spring force vector 
    Vec2 springForce = springDirection * springMagnitude;
    return springForce;
}

Vec2 Force::GenerateSpringForce(const Body& a, const Body& b, float restLength, float k) {
    // calculate the distance between 2 bodies
    Vec2 d = a.position - b.position;
    
    // find the spring displacement considering the rest length 
    float displacement = d.Magnitude() - restLength;

    // calculate the direction and the magitude of the spring force 
    Vec2 springDirection = d.UnitVector();
    float springMagnitude = -k * displacement;

    // calculate the final resulting spring force vector 
    Vec2 springForce = springDirection * springMagnitude;
    return springForce;
}
