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
    return new PolygonShape(vertices);
}

BoxShape::BoxShape(float width, float height) {
    // TODO
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
