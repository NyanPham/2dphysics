#include "Application.h"
#include "./Physics/Constants.h"
#include "./Physics/Force.h"
#include "Physics/CollisionDetection.h"
#include "Physics/Constraint.h"
#include "Physics/Contact.h"
#include "Physics/Shape.h"
#include <SDL2/SDL_mouse.h>

bool Application::IsRunning() {
    return running;
}

// Setup function (executed once in the beginning)
void Application::Setup() {
    running = Graphics::OpenWindow();
    
    world = new World(-9.8);
   
    // Body* bigBall = new Body(CircleShape(64), Graphics::Width() / 2.0, Graphics::Height() / 2.0, 0.0);
    // bigBall->SetTexture("./assets/bowlingball.png");
    // world->AddBody(bigBall);

    Body* floor = new Body(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2.0, Graphics::Height() - 50, 0.0);
    Body* leftWall = new Body(BoxShape(50, Graphics::Height() - 100), 50, Graphics::Height() / 2.0 - 25, 0.0);
    Body* rightWall = new Body(BoxShape(50, Graphics::Height() - 100), Graphics::Width() - 50, Graphics::Height() / 2.0 - 25, 0.0);
    floor->restitution = 0.7;
    leftWall->restitution = 0.2;
    rightWall->restitution = 0.2;
    world->AddBody(floor);
    world->AddBody(leftWall);
    world->AddBody(rightWall);

    Body* a = new Body(BoxShape(200, 200), Graphics::Width() / 2.0, Graphics::Height() / 2.0, 0.0);
    Body* b = new Body(BoxShape(200, 200), 300, 0, 0.0);
    a->rotation = 0.0;
    b->rotation = 0.0;
    world->AddBody(a);
    world->AddBody(b);

    // Body* bob = new Body(CircleShape(5), Graphics::Width() / 2.0, Graphics::Height() / 2.0 - 200, 0.0);
    // Body* head = new Body(CircleShape(25), bob->position.x, bob->position.y + 70, 5.0);
    // Body* torso = new Body(BoxShape(50, 100), head->position.x, head->position.y + 80, 3.0);
    // Body* leftArm = new Body(BoxShape(15, 70), torso->position.x - 32, torso->position.y - 10, 1.0);
    // Body* rightArm = new Body(BoxShape(15, 70), torso->position.x + 32, torso->position.y - 10, 1.0);
    // Body* leftLeg = new Body(BoxShape(20, 90), torso->position.x - 20, torso->position.y + 97, 1.0);
    // Body* rightLeg = new Body(BoxShape(20, 90), torso->position.x + 20, torso->position.y + 97, 1.0);
    // bob->SetTexture("./assets/ragdoll/bob.png");
    // head->SetTexture("./assets/ragdoll/head.png");
    // torso->SetTexture("./assets/ragdoll/torso.png");
    // leftArm->SetTexture("./assets/ragdoll/leftArm.png");
    // rightArm->SetTexture("./assets/ragdoll/rightArm.png");
    // leftLeg->SetTexture("./assets/ragdoll/leftLeg.png");
    // rightLeg->SetTexture("./assets/ragdoll/rightLeg.png");
    // world->AddBody(bob);
    // world->AddBody(head);
    // world->AddBody(torso);
    // world->AddBody(leftArm);
    // world->AddBody(rightArm);
    // world->AddBody(leftLeg);
    // world->AddBody(rightLeg);
    //
    // JointConstraint* string = new JointConstraint(bob, head, bob->position);
    // JointConstraint* neck = new JointConstraint(head, torso, head->position + Vec2(0, 25));
    // JointConstraint* leftShoulder = new JointConstraint(torso, leftArm, torso->position + Vec2(-28, -45));
    // JointConstraint* rightShoulder = new JointConstraint(torso, rightArm, torso->position + Vec2(28, -45));
    // JointConstraint* leftHip = new JointConstraint(torso, leftLeg, torso->position + Vec2(-20, 50));
    // JointConstraint* rightHip = new JointConstraint(torso, rightLeg, torso->position + Vec2(20, 50));
    // world->AddConstraint(string);
    // world->AddConstraint(neck);
    // world->AddConstraint(leftShoulder);
    // world->AddConstraint(rightShoulder);
    // world->AddConstraint(leftHip);
    // world->AddConstraint(rightHip);
    //
    // Body* floor = new Body(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2.0, Graphics::Height() - 50, 0.0);
    // Body* leftWall = new Body(BoxShape(50, Graphics::Height() - 100), 50, Graphics::Height() / 2.0 - 25, 0.0);
    // Body* rightWall = new Body(BoxShape(50, Graphics::Height() - 100), Graphics::Width() - 50, Graphics::Height() / 2.0 - 25, 0.0);
    // floor->restitution = 0.7;
    // leftWall->restitution = 0.2;
    // rightWall->restitution = 0.2;
    // world->AddBody(floor);
    // world->AddBody(leftWall);
    // world->AddBody(rightWall);
}
// Input processing
void Application::Input() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                if (event.key.keysym.sym == SDLK_d) {
                    debug = !debug;
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    Body* ball = new Body(CircleShape(30), x, y, 1.0);
                    ball->SetTexture("./assets/basketball.png");
                    ball->restitution = 0.7;
                    world->AddBody(ball);
                }
                if (event.button.button == SDL_BUTTON_RIGHT) {
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    Body* box = new Body(BoxShape(60, 60), x, y, 1.0);
                    box->SetTexture("./assets/crate.png");
                    box->restitution = 0.2;
                    world->AddBody(box);
                }
                break;
            case SDL_MOUSEMOTION:
                int x, y; 
                SDL_GetMouseState(&x, &y);
                Body* box = world->GetBodies()[4];
                box->position.x = x;
                box->position.y = y;
                break;
        }
    }
}

// Update function (called each frame)
void Application::Update() {
    Graphics::ClearScreen(0xFF0F0721);

    // wait some time until the target frame time is reached
    static int timePreviousFrame;
    int timeToWait = MILLISECS_PER_FRAME - (SDL_GetTicks() - timePreviousFrame);
    if (timeToWait > 0) 
        SDL_Delay(timeToWait);

    // calculate the deltatime in seconds
    float deltaTime = (SDL_GetTicks() - timePreviousFrame) / 1000.0f;
    if (deltaTime > 0.016) {
        deltaTime = 0.016;
    }

    // set the time of the current frame to be used in the next frame
    timePreviousFrame = SDL_GetTicks();
   
    // update world bodies (integration, collision detection, etc.)
    world->Update(deltaTime);
}

// Render function (called each frame)
void Application::Render() {
    // Body* bob = world->GetBodies()[0];
    // Body* head = world->GetBodies()[1];
    // Graphics::DrawLine(bob->position.x, bob->position.y, head->position.x, head->position.y, 0xFF555555);
    //
    // for (auto joint: world->GetConstraints()) {
    //     if (debug) {
    //         const Vec2 anchorPoint = joint->a->LocalSpaceToWorldSpace(joint->aPoint);
    //         Graphics::DrawFillCircle(anchorPoint.x, anchorPoint.y, 3, 0xFF0000FF);
    //     }
    // }

    for (auto body: world->GetBodies()) {
        if (body->shape->GetType() == CIRCLE) {
            CircleShape* circleShape = (CircleShape*) body->shape;
            if (!debug && body->texture) {
                Graphics::DrawTexture(body->position.x, body->position.y, circleShape->radius * 2, circleShape->radius * 2, body->rotation, body->texture);
            } else {
                Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFF00FF00);
            }
        } 
        if (body->shape->GetType() == BOX) {
            BoxShape* boxShape = (BoxShape*) body->shape;
            if (!debug && body->texture) {
                Graphics::DrawTexture(body->position.x, body->position.y, boxShape->width, boxShape->height, body->rotation, body->texture); 
            } else {
                Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFF00FF00);
            }
        }
        if (body->shape->GetType() == POLYGON) {
            PolygonShape* polygonShape = (PolygonShape*) body->shape;
            if (!debug) {
                Graphics::DrawFillPolygon(body->position.x, body->position.y, polygonShape->worldVertices, 0xFF444444);
            } else {
                Graphics::DrawPolygon(body->position.x, body->position.y, polygonShape->worldVertices, 0xFF00FF00);
            }
        }
    }

    Graphics::RenderFrame();
}

// Destroy function to delete objects and close the window
void Application::Destroy() {
    delete world;
    Graphics::CloseWindow();
}
