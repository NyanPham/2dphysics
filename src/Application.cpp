#include "Application.h"
#include "./Physics/Constants.h"
#include "./Physics/Force.h"
#include "Physics/CollisionDetection.h"
#include "Physics/Contact.h"
#include <SDL2/SDL_mouse.h>

bool Application::IsRunning() {
    return running;
}

// Setup function (executed once in the beginning)
void Application::Setup() {
    running = Graphics::OpenWindow();

    Body* bigBall = new Body(CircleShape(100), 100, 100, 0.0);
    Body* smallBall = new Body(CircleShape(50), 500, 100, 1.0);
    bodies.push_back(bigBall);
    bodies.push_back(smallBall);
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
                if (event.key.keysym.sym == SDLK_UP) {
                    pushForce.y = -50 * PIXELS_PER_METER;
                }
                if (event.key.keysym.sym == SDLK_RIGHT) {
                    pushForce.x = 50 * PIXELS_PER_METER;
                }
                if (event.key.keysym.sym == SDLK_DOWN) {
                    pushForce.y = 50 * PIXELS_PER_METER;
                }
                if (event.key.keysym.sym == SDLK_LEFT) {
                    pushForce.x = -50 * PIXELS_PER_METER;
                }
                break;
            case SDL_KEYUP:
                if (event.key.keysym.sym == SDLK_UP) {
                    pushForce.y = 0;
                }
                if (event.key.keysym.sym == SDLK_RIGHT) {
                    pushForce.x = 0;
                }
                if (event.key.keysym.sym == SDLK_DOWN) {
                    pushForce.y = 0;
                }
                if (event.key.keysym.sym == SDLK_LEFT) {
                    pushForce.x = 0; 
                }
                break;
            case SDL_MOUSEMOTION:
                int x, y;
                SDL_GetMouseState(&x, &y);
                bodies[0]->position.x = x;
                bodies[0]->position.y = y;
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

    // apply forces to the bodies  
    for (auto body: bodies) {
        body->AddForce(pushForce);

        // apply the weight force 
        // Vec2 weight = Vec2(0.0, body->mass * 9.8 * PIXELS_PER_METER);
        // body->AddForce(weight);
        //
        // Vec2 wind = Vec2(20.0 * PIXELS_PER_METER, 0.0);
        // body->AddForce(wind);

        // float torque = 200;
        // body->AddTorque(torque);
    }

    // integrate the acceleration and the velocity to find the new position
    for (auto body: bodies) {
        body->Update(deltaTime);
    }

    // check all the rigidbodies with other rigidbodies for collision
    for (size_t i = 0; i < bodies.size() - 1; i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            Body* a = bodies[i];
            Body* b = bodies[j];
        
            a->isColliding = false;
            b->isColliding = false;
            Contact contact;
            if (CollisionDetection::IsColliding(a, b, contact)) {
                Graphics::DrawFillCircle(contact.start.x, contact.start.y, 3, 0xFFFF00FF);
                Graphics::DrawFillCircle(contact.end.x, contact.end.y, 3, 0xFFFF00FF);
                Graphics::DrawLine(contact.start.x, contact.start.y, contact.start.x + contact.normal.x * 15, contact.start.y + contact.normal.y * 15, 0xFFFF00FF);

                a->isColliding = true;
                b->isColliding = true;

                contact.ResolvePenetration();
            } 
        }
    }
    
    // check the boundary of the window
    for (auto body: bodies) {
        if (body->shape->GetType() == CIRCLE) {
            CircleShape* circleShape = (CircleShape*) body->shape;
            if (body->position.x - circleShape->radius <= 0) {
                body->position.x = circleShape->radius;
                body->velocity.x *= -0.9;
            } else if (body->position.x + circleShape->radius >= Graphics::Width()) {
                body->position.x = Graphics::Width() - circleShape->radius;
                body->velocity.x *= - 0.9;
            }
            if (body->position.y - circleShape->radius <= 0) {
                body->position.y = circleShape->radius;
                body->velocity.y *= -0.9;
            } else if (body->position.y + circleShape->radius >= Graphics::Height()) {
                body->position.y = Graphics::Height() - circleShape->radius;
                body->velocity.y *= -0.9;
            }
        }
    }
}

// Render function (called each frame)
void Application::Render() {

    for (auto body: bodies) {
        uint32_t color = body->isColliding ? 0xFF0000FF : 0xFFFFFFFF;

        if (body->shape->GetType() == CIRCLE) {
            CircleShape* circleShape = (CircleShape*) body->shape;
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, color);
        } 
        if (body->shape->GetType() == BOX) {
            BoxShape* boxShape = (BoxShape*) body->shape;
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);
        }
    }

    Graphics::RenderFrame();
}

// Destroy function to delete objects and close the window
void Application::Destroy() {
    for (auto body: bodies) {
        delete body;
    }
    Graphics::CloseWindow();
}
