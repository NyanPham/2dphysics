#include "./Shape.h"
#include <iostream>
#include <limits>

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

void CircleShape::UpdateVertices(float angle, const Vec2& position) {
    return; // circles don't have vertices... nothing to do here
}

float CircleShape::GetMomentOfInertia() const {
    // for solid circles, the moment of inertia is 1/2 * r^2 
    // but this still needs to be multiplied by the rigidbody's mass 
    return 0.5 * (radius * radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2> vertices) {
    for (auto vertex: vertices) {
        localVertices.push_back(vertex);
        worldVertices.push_back(vertex);
    }
    std::cout << "PolygonShape constructor called!\n"; 
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
    float acc0 = 0;
    float acc1 = 0;
    for (size_t i = 0; i < localVertices.size(); i++) {
        auto a = localVertices[i];
        auto b = localVertices[(i + 1) % localVertices.size()];
        auto cross = abs(a.Cross(b));
        acc0 += cross * (a.Dot(a) + b.Dot(b) + a.Dot(b));
        acc1 += cross;
    }
    return acc0 / 6 / acc1;
}

Vec2 PolygonShape::EdgeAt(int index) const {
    int currVertex = index;
    int nextVertex = (index + 1) % worldVertices.size();

    return worldVertices[nextVertex] - worldVertices[currVertex];
}

float PolygonShape::FindMinSeparation(const PolygonShape* other, int& indexReferenceEdge, Vec2& supportPoint) const {
    float separation = std::numeric_limits<float>::lowest();

    // loop all the vertices of "this" polygon
    for (size_t i = 0; i < this->worldVertices.size(); i++) {
        Vec2 va = this->worldVertices[i];
        Vec2 normal = this->EdgeAt(i).Normal();

        float minSep = std::numeric_limits<float>::max();
        Vec2 minVertex;

        // loop all the vertices of "other" polygon
        for (size_t j = 0; j < other->worldVertices.size(); j++){
            Vec2 vb = other->worldVertices[j];
            float proj = (vb - va).Dot(normal);
            
            if (proj < minSep) {
                minSep = proj;
                minVertex = vb; 
            }
        }
       
        if (minSep > separation) {
            separation = minSep;
            indexReferenceEdge = i;
            supportPoint = minVertex;
        }
    }

    return separation;
}

int PolygonShape::FindIncidentEdge(const Vec2& normal) const {
    int indexIncidentEdge;
    float minProj = std::numeric_limits<float>::max();
    for (size_t i = 0; i < this->worldVertices.size(); i++) {
        auto edgeNormal = this->EdgeAt(i).Normal();
        auto proj = edgeNormal.Dot(normal);
        if (proj < minProj) {
            minProj = proj;
            indexIncidentEdge = (int)i;
        }
    }

    return indexIncidentEdge;
}


int PolygonShape::ClipSegmentToLine(const std::vector<Vec2>& contactsIn, std::vector<Vec2>& contactsOut, const Vec2& c0, const Vec2& c1) const {
    // start with no output points 
    int numOut = 0;

    // calculate the distance of end points to the line 
    Vec2 normal = (c1 - c0).Normalize();
    float dist0 = (contactsIn[0] - c0).Cross(normal);
    float dist1 = (contactsIn[1] - c0).Cross(normal); 
    
    // if the points are behind the plane 
    if (dist0 <= 0)
        contactsOut[numOut++] = contactsIn[0];
    if (dist1 <= 0)
        contactsOut[numOut++] = contactsIn[1];

    // if the points are on different sides of the plane (one distance is negative and the other is postive) {
    if (dist0 * dist1 < 0) {
        float totalDist = dist0 - dist1; 

        // find the intersection using linear interpolation: lerp(start, end) => start + t*(end-start)
        float t = dist0 / totalDist;
        Vec2 contact = contactsIn[0] + (contactsIn[1] - contactsIn[0]) * t;
        contactsOut[numOut] = contact; 
        numOut++;
    }
    return numOut;
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
