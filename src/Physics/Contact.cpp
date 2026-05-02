#include "Contact.h"

// resolves the collisions using the projection method
void Contact::ResolvePenetration() {
    if (a->IsStatic() && b->IsStatic()) {
        return;
    }

    float da = depth / (a->invMass + b->invMass) * a->invMass;
    float db = depth / (a->invMass + b->invMass) * b->invMass;

    a->position -= normal * da;
    b->position += normal * db;
}

// resolves the collision using the impulse method
void Contact::ResolveCollision() {
    // apply positional correction using the projection method
    ResolvePenetration();

    // define the elasticity (coefficient of restituion e)
    float e = std::min(a->restitution, b->restitution);
    float f = std::min(a->friction, b->friction);

    // calculate the relative velocity between the two objects
    Vec2 ra = end - a->position;
    Vec2 rb = start - b->position;
    Vec2 va = a->velocity + Vec2(-a->angularVelocity * ra.y, a->angularVelocity * ra.x);
    Vec2 vb = b->velocity + Vec2(-b->angularVelocity * rb.y, b->angularVelocity * rb.x);

    const Vec2 vrel = va - vb;

    // now we proceed to calculate the collision impulse
    float vrelDotNormal = vrel.Dot(normal);
    const Vec2 impulseDirectionN = normal;
    const float impulseMagnitudeN = -(1 + e) * vrelDotNormal / ((a->invMass + b->invMass) + (ra.Cross(normal) * ra.Cross(normal) * a->invI) + (rb.Cross(normal) * rb.Cross(normal) * b->invI));
    Vec2 jN = impulseDirectionN * impulseMagnitudeN;
    
    // now we calculate the collision impulse along the tangent
    Vec2 tangent = normal.Normal();
    float vrelDotTangent = vrel.Dot(tangent);
    const Vec2 impulseDirectionT = tangent;
    const float impulseMagnitudeT = f * -(1 + e) * vrelDotTangent / ((a->invMass + b->invMass) + (ra.Cross(tangent) * ra.Cross(tangent) * a->invI) + (rb.Cross(tangent) * rb.Cross(tangent) * b->invI));
    Vec2 jT = impulseDirectionT * impulseMagnitudeT; 

    // calculate the final impulse j combining normal and tangent impulses
    Vec2 j = jN + jT;

    // apply the impulse vector to both objects in opposite direction 
    a->ApplyImpulse(j, ra);
    b->ApplyImpulse(-j, rb);
}
