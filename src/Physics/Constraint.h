#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Body.h"
#include "MatMN.h" 

class Constraint {
    public:
        Body* a;
        Body* b;
        
        Vec2 aPoint;    // the anchor point in A's local space 
        Vec2 bPoint;    // the anchor point in B's local space

        virtual ~Constraint() = default;
        
        MatMN GetInvM() const;
        VecN GetVelocities() const;

        virtual void Solve(void) {}
        virtual void PreSolve(const float dt) {}
        virtual void PostSolve(void) {}
};

class JointConstraint: public Constraint {
    private:
        MatMN jacobian;
        VecN cachedLambda;
        float bias;
        
    public:
        JointConstraint();
        JointConstraint(Body* a, Body* b, const Vec2& anchorPoint);
        void Solve() override;
        void PreSolve(const float dt) override;
        void PostSolve() override;
};

class PenetrationConstraint: public Constraint {
    private: 
        MatMN jacobian;
        VecN cachedLambda;
        float bias;
        Vec2 normal;        // normal direction of the penetration in A's local space 
        float friction;     // frcition coefficient between the two penetrating bodies

    public:
        PenetrationConstraint();
        PenetrationConstraint(Body* a, Body* b, const Vec2& aCollisionPoint, const Vec2& bCollisionPoint, const Vec2& normal);
        void Solve() override;
        void PreSolve(const float dt) override;
        void PostSolve() override;
}; 

#endif
