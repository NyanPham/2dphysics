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
    
    Body* a = new Body(CircleShape(30), Graphics::Width() / 2.0, Graphics::Height() / 2.0, 0.0f);
    Body* b = new Body(CircleShape(20), a->position.x - 100, a->position.y, 1.0f);

    world->AddBody(a);
    world->AddBody(b);

    JointConstraint* joint = new JointConstraint(a, b, a->position);
    world->AddConstraint(joint);
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
                int x, y;
                SDL_GetMouseState(&x, &y);
                if (event.button.button == SDL_BUTTON_LEFT) {
                    Body* ball = new Body(CircleShape(30), x, y, 1.0);
                    ball->SetTexture("./assets/basketball.png");
                    ball->restitution = 0.7;
                    world->AddBody(ball);
                }
                if (event.button.button == SDL_BUTTON_RIGHT) {
                    Body* box = new Body(BoxShape(60, 60), x, y, 1.0);
                    box->SetTexture("./assets/crate.png");
                    box->restitution = 0.2;
                    world->AddBody(box);
                }
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
