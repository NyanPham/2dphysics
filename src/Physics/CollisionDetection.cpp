#include "CollisionDetection.h"
#include "Contact.h"
#include "Shape.h"
#include "../Graphics.h"

bool CollisionDetection::IsColliding(Body* a, Body* b, std::vector<Contact>&contacts) {
    bool aIsCircle = a->shape->GetType() == CIRCLE;
    bool bIsCircle = b->shape->GetType() == CIRCLE;
    bool aIsPolygon = a->shape->GetType() == POLYGON || a->shape->GetType() == BOX;
    bool bIsPolygon = b->shape->GetType() == POLYGON || b->shape->GetType() == BOX;

    if (aIsCircle && bIsCircle) {
        return IsCollidingCircleCircle(a, b, contacts);
    }
    
    if (aIsPolygon && bIsPolygon) {
        return IsCollidingPolygonPolygon(a, b, contacts);
    }

    if (aIsPolygon && bIsCircle) {
        return IsCollidingPolygonCircle(a, b, contacts);
    }

    if (aIsCircle && bIsPolygon) {
        return IsCollidingPolygonCircle(b, a, contacts);
    }

    return false;
}

bool CollisionDetection::IsCollidingCircleCircle(Body *a, Body *b, std::vector<Contact>& contacts) {
    CircleShape* aCircleShape = (CircleShape*) a->shape;
    CircleShape* bCircleShape = (CircleShape*) b->shape;
    
    const Vec2 ab = b->position - a->position;
    const float radiusSum = aCircleShape->radius + bCircleShape->radius;

    bool isColliding = ab.MagnitudeSquared() <= (radiusSum * radiusSum);
    if (!isColliding) {
        return false;
    }
    
    Contact contact;
    contact.a = a;
    contact.b = b;
    
    contact.normal = ab;
    contact.normal.Normalize();
    
    contact.start = b->position - (contact.normal * bCircleShape->radius);
    contact.end = a->position + (contact.normal * aCircleShape->radius); 

    contact.depth = (contact.end - contact.start).Magnitude();
    
    contacts.push_back(contact);
    return true;
}

bool CollisionDetection::IsCollidingPolygonPolygon(Body *a, Body *b, std::vector<Contact>& contacts) {
    PolygonShape* aPolygonShape = (PolygonShape*) a->shape;
    PolygonShape* bPolygonShape = (PolygonShape*) b->shape;
   
    int aIndexReferenceEdge, bIndexReferenceEdge;
    Vec2 aSupportPoint, bSupportPoint;
    float abSeparation = aPolygonShape->FindMinSeparation(bPolygonShape, aIndexReferenceEdge, aSupportPoint);
    if (abSeparation >= 0) {
        return false;
    }
    float baSeparation = bPolygonShape->FindMinSeparation(aPolygonShape, bIndexReferenceEdge, bSupportPoint);
    if (baSeparation >= 0) {
        return false;
    }
   
    PolygonShape* referenceShape;
    PolygonShape* incidentShape;
    int indexReferenceEdge;
    if (abSeparation > baSeparation) {
        referenceShape = aPolygonShape;
        incidentShape = bPolygonShape;
        indexReferenceEdge = aIndexReferenceEdge;
    } else {
        referenceShape = bPolygonShape;
        incidentShape = aPolygonShape;
        indexReferenceEdge = bIndexReferenceEdge;
    }

    // find the reference edge based on the index that returned from the function
    Vec2 referenceEdge = referenceShape->EdgeAt(indexReferenceEdge);       
    
    // find the incident edge
    // for CLIPPING
    int incidentIndex = incidentShape->FindIncidentEdge(referenceEdge.Normal());
    int incidentNextIndex = (incidentIndex + 1) % incidentShape->worldVertices.size();
    Vec2 v0 = incidentShape->worldVertices[incidentIndex];
    Vec2 v1 = incidentShape->worldVertices[incidentNextIndex];
    
    std::vector<Vec2> contactPoints = {v0,v1};
    std::vector<Vec2> clippedPoints = contactPoints;
    for (size_t i = 0; i < referenceShape->worldVertices.size(); i++) {
        if ((int)i == indexReferenceEdge)
            continue;
        Vec2 c0 = referenceShape->worldVertices[i];
        Vec2 c1 = referenceShape->worldVertices[(i + 1) % referenceShape->worldVertices.size()];
        int numClipped = referenceShape->ClipSegmentToLine(contactPoints, clippedPoints, c0, c1);
        if (numClipped < 2) 
            break;
        
        contactPoints = clippedPoints;  // make the next contact points the ones that were just clipped 
    }
    
    auto vref = referenceShape->worldVertices[indexReferenceEdge];

    // loop all clipped points, but only consider those where separation is negative 
    for (auto& vclip: clippedPoints) {
        float separation = (vclip - vref).Dot(referenceEdge.Normal());
        if (separation <= 0) {
            Contact contact; 
            contact.a = a;
            contact.b = b;
            contact.normal = referenceEdge.Normal();
            contact.start = vclip; 
            contact.end = vclip + contact.normal * -separation;
            if (baSeparation >= abSeparation) {
                std::swap(contact.start, contact.end);  // the start-end points are always from 'a' to 'b'
                contact.normal *= -1.0;                 // the collision normal is always from 'a' to 'b'
            }

            contacts.push_back(contact);
        }
    }
    return true;
}

bool CollisionDetection::IsCollidingPolygonCircle(Body *polygon, Body *circle, std::vector<Contact>& contacts) {
    const PolygonShape* polygonShape = (PolygonShape*) polygon->shape;
    const CircleShape* circleShape = (CircleShape*) circle->shape;
    const std::vector<Vec2>& polygonVertices = polygonShape->worldVertices;

    bool isOutside = false;
    Vec2 minCurrVertex;
    Vec2 minNextVertex;
    float distanceCircleEdge = std::numeric_limits<float>::lowest();

    // loop all the edges of the polygon/box 
    for (size_t i = 0; i < polygonVertices.size(); i++) {
        size_t currVertex = i;
        size_t nextVertex = (i + 1) % polygonVertices.size();
        Vec2 edge = polygonShape->EdgeAt(currVertex);
        Vec2 normal = edge.Normal();
        
        // compare the circle center with the polygon vertex 
        Vec2 circleCenter = circle->position - polygonVertices[currVertex];

        // project the circle center onto the edge normal 
        float projection = circleCenter.Dot(normal);

        if (projection > 0) {
            // circle center is outside the polygon 
            distanceCircleEdge = projection;
            minCurrVertex = polygonVertices[currVertex];
            minNextVertex = polygonVertices[nextVertex];
            isOutside = true;
            break;
        } else {
            // circle is inside the polygon, find the min edge (least negative projection)
            if (projection > distanceCircleEdge) {
                distanceCircleEdge = projection;
                minCurrVertex = polygonVertices[currVertex];
                minNextVertex = polygonVertices[nextVertex];
            }
        }
    }
    
    Contact contact;

    if (isOutside) {
        /////////////////////////////////////
        // check if we're inside region A:
        // first vertex is inside the circle
        /////////////////////////////////////
        Vec2 v1 = circle->position - minCurrVertex; // vector from the nearest vertex to the circle vertex 
        Vec2 v2 = minNextVertex - minCurrVertex;    // the nearest edge (from curr vertex to next vertex)
        if (v1.Dot(v2) < 0) {
            // distance from vertex to cirle centeris greater than radius... no collision 
            if (v1.Magnitude() > circleShape->radius) {
                return false;
            } else {
                // collision detected in region A 
                contact.a = polygon;
                contact.b = circle;
                contact.depth = circleShape->radius - v1.Magnitude();
                contact.normal = v1.Normalize();
                contact.start = circle->position + (contact.normal * -circleShape->radius);
                contact.end = contact.start + (contact.normal * contact.depth);
            }
        } else {
            /////////////////////////////////////
            // check if we're inside region B:
            // second vertex is inside the circle
            /////////////////////////////////////
             
            v1 = circle->position - minNextVertex; // vector from the nearest vertex to the circle center 
            v2 = minCurrVertex - minNextVertex;    // the nearest edge
            if (v1.Dot(v2) < 0) {
                // distance from vertex to cirle center is greater than radius... no collision 
                if (v1.Magnitude() > circleShape->radius) {
                    return false;
                }else {
                    // collision detected in region B 
                    contact.a = polygon;
                    contact.b = circle; 
                    contact.depth = circleShape->radius - v1.Magnitude();
                    contact.normal = v1.Normalize();
                    contact.start = circle->position + (contact.normal * -circleShape->radius); 
                    contact.end = contact.start + (contact.normal * contact.depth);
                }
            } else {
                // check if we're inside region C:
                // no vertices are inside the circle
                /////////////////////////////////////
                if (distanceCircleEdge > circleShape->radius) {
                    // no colliison... distance between the closest distance and the circle center is greater than the radius 
                    return false;
                } else {
                    // collision detected in region C 
                    contact.a = polygon;
                    contact.b = circle; 
                    contact.depth = circleShape->radius - distanceCircleEdge;
                    contact.normal = (minNextVertex - minCurrVertex).Normal();
                    contact.start = circle->position - (contact.normal * circleShape->radius);
                    contact.end = contact.start + (contact.normal * contact.depth);
                }
            }
        }
    } else {
        // the center of the circle is inside the polygon... it is definitely colliding! 
        contact.a = polygon;
        contact.b = circle;
        contact.depth = circleShape->radius - distanceCircleEdge; 
        contact.normal = (minNextVertex - minCurrVertex).Normal();
        contact.start = circle->position - (contact.normal * circleShape->radius);
        contact.end = contact.start + (contact.normal * contact.depth);
    }
    
    contacts.push_back(contact);
    return true;
}
