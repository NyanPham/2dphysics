#include "./Force.h"

Vec2 Force::GenerateDragForce(const Particle& particle, float k) {
    Vec2 dragForce = Vec2(0, 0);

    if (particle.velocity.MagnitudeSquared() > 0) {
        // calculate the drag direction (inverse of the velocity unit vector)
        Vec2 dragDirection = particle.velocity.UnitVector() * -1.0;

        // calculate the drag magnitude, k * |v|^2 
        float dragMagnitude = k * particle.velocity.MagnitudeSquared();

        // generate the final drag force with direction and magnitude
        dragForce = dragDirection * dragMagnitude;
    }
    return dragForce;
}
