#include "./Shape.h"
#include <iostream>

CircleShape::CircleShape(float radius) {
    this->radius = radius;
    std::cout << "CircleShape constructor called!\n"; 
}

CircleShape::~CircleShape() {
    std::cout << "CircleShape destructor called!\n"; 
}

ShapeType CircleShape::GetType() const {
    return CIRCLE;
}

Shape* CircleShape::Clone() const {
    return new CircleShape(radius);
}

float CircleShape::GetMomentOfInertia() const {
    // for solid circles, the moment of inertia is 1/2 * r^2 
    // but this still needs to be multiplied by the rigidbody's mass 
    return 0.5 * (radius * radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2> vertices) {
    // TODO
}

PolygonShape::~PolygonShape() {
    std::cout << "PolygonShape destructor called!\n"; 
}

ShapeType PolygonShape::GetType() const {
    return POLYGON;
}

Shape* PolygonShape::Clone() const {
    return new PolygonShape(localVertices);
}

float PolygonShape::GetMomentOfInertia() const {
    return 0.0;
}

void PolygonShape::UpdateVertices(float angle, const Vec2& position) {
    // loop all the vertices, transforming from local to world space
    for (size_t i = 0; i < localVertices.size(); i++) {
        // rotate -> translate
        worldVertices[i] = localVertices[i].Rotate(angle);
        worldVertices[i] += position;
    }
}

BoxShape::BoxShape(float width, float height) {
    this->width = width;
    this->height = height;
    
    // load the vertices of the box PolygonShape
    localVertices.push_back(Vec2(-width / 2.0, -height / 2.0));
    localVertices.push_back(Vec2(width / 2.0, -height / 2.0));
    localVertices.push_back(Vec2(width / 2.0, height / 2.0));
    localVertices.push_back(Vec2(-width / 2.0, height / 2.0));

    worldVertices.push_back(Vec2(-width / 2.0, -height / 2.0));
    worldVertices.push_back(Vec2(width / 2.0, -height / 2.0));
    worldVertices.push_back(Vec2(width / 2.0, height / 2.0));
    worldVertices.push_back(Vec2(-width / 2.0, height / 2.0));
}

BoxShape::~BoxShape() {
    std::cout << "BoxShape destructor called!\n"; 
}

ShapeType BoxShape::GetType() const {
    return BOX;
}

Shape* BoxShape::Clone() const {
    return new BoxShape(width, height);
}

float BoxShape::GetMomentOfInertia() const {
    // for a rectangle, the moment of inertai is 1/12 * (w^2 + h^2)
    // but this still needs to be multiplied by the rigidbody's mass
    return (0.083333) * (width * width + height * height);
}
