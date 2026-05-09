#include "Constraint.h"

////////////////////////////////////////////////////////////////////////////
// Mat6x6 with the all inverse mass and inversion I of bodies "a" and "b"
//////////////////////////////////////////////////////////////////////////// 
//
// [    1/ma    0       0       0       0       0       ]
// [      0     1/ma    0       0       0       0       ]
// [      0     0       1/Ia    0       0       0       ]
// [      0     0       0       1/mb    0       0       ]
// [      0     0       0       0       1/mb    0       ]
// [      0     0       0       0       1/Ib    0       ]
//
//////////////////////////////////////////////////////////////////////////// 
MatMN Constraint::GetInvM() const {
    MatMN invM(6, 6); 
    invM.Zero(); 
    invM.rows[0][0] = a->invMass;
    invM.rows[1][1] = a->invMass;
    invM.rows[2][2] = a->invI;
    invM.rows[3][3] = b->invMass;
    invM.rows[4][4] = b->invMass;
    invM.rows[5][5] = b->invI;
    return invM;
}

//////////////////////////////////////////////////////////////////////////// 
// Vec6 with the all the linear and angular velocities of bodies "a" and "b"
//////////////////////////////////////////////////////////////////////////// 
//
// [ va.x ]
// [ va.y ]
// [ Wa   ]
// [ vb.x ]
// [ vb.y ]
// [ Wb   ]
//
//////////////////////////////////////////////////////////////////////////// 
VecN Constraint::GetVelocities() const {
    VecN V(6);
    V.Zero();
    V[0] = a->velocity.x;
    V[1] = a->velocity.y;
    V[2] = a->angularVelocity;
    V[3] = b->velocity.x;
    V[4] = b->velocity.y;
    V[5] = b->angularVelocity;
    return V;
}

JointConstraint::JointConstraint(): Constraint(), jacobian(1, 6), cachedLambda(1), bias(0.0f) {
   cachedLambda.Zero(); 
}

JointConstraint::JointConstraint(Body* a, Body* b, const Vec2& anchorPoint): Constraint(), jacobian(1, 6), cachedLambda(1), bias(0.0f) {
    this->a = a;
    this->b = b;
    this->aPoint = a->WorldSpaceToLocalSpace(anchorPoint);
    this->bPoint = b->WorldSpaceToLocalSpace(anchorPoint);
    cachedLambda.Zero();
}

void JointConstraint::PreSolve(const float dt) {
    // get the anchor point position in world space 
    const Vec2 pa = a->LocalSpaceToWorldSpace(aPoint);
    const Vec2 pb = b->LocalSpaceToWorldSpace(bPoint);

    const Vec2 ra = pa - a->position;
    const Vec2 rb = pb - b->position;

    Vec2 J1 = (pa - pb) * 2.0;
    jacobian.rows[0][0] = J1.x; // A linear velocity.x
    jacobian.rows[0][1] = J1.y; // A linear velocity.y 

    float J2 = ra.Cross(pa - pb) * 2.0f;
    jacobian.rows[0][2] = J2;   // A angular velocity 

    Vec2 J3 = (pb - pa) * 2.0;
    jacobian.rows[0][3] = J3.x; // B linear velocity.x
    jacobian.rows[0][4] = J3.y; // B linear velocity.y 

    float J4 = rb.Cross(pb - pa) * 2.0f;
    jacobian.rows[0][5] = J4;   // B angular velocity 
    
    // wam starting (apply cached lambda)
    const MatMN Jt = jacobian.Transpose();
    VecN impulses = Jt * cachedLambda;

    // apply the impulses to both A and B 
    a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
    a->ApplyImpulseAngular(impulses[2]);
    b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
    b->ApplyImpulseAngular(impulses[5]);

    // compute the bias term (baumgarte stabilization)
    const float beta = 0.2f;
    float C = (pb - pa).Dot(pb - pa);
    C = std::max(0.0f, C - 0.01f);
    bias = (beta / dt) * C; 
}

void JointConstraint::Solve() {
    const VecN V = GetVelocities(); 
    const MatMN invM = GetInvM();
   
    const MatMN J = jacobian;
    const MatMN Jt = jacobian.Transpose();
    
    // calculate the numerator 
    MatMN lhs = J * invM * Jt;
    VecN rhs = J * V * -1.0f;
    rhs[0] -= bias;

    VecN lambda = MatMN::SolveGaussSeidel(lhs, rhs);
    cachedLambda += lambda;

    // compute the final impulses with direction and magnitude 
    VecN impulses = Jt * lambda;

    // apply the impulses to both A and B 
    a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
    a->ApplyImpulseAngular(impulses[2]);
    b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
    b->ApplyImpulseAngular(impulses[5]);
}

void JointConstraint::PostSolve() {

}
