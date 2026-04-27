#include "Application.h"
#include "./Physics/Constants.h"
#include "./Physics/Force.h"

bool Application::IsRunning() {
    return running;
}

// Setup function (executed once in the beginning)
void Application::Setup() {
    running = Graphics::OpenWindow();
    
    // Particle* smallBall= new Particle(50, 100, 1.0);
    // smallBall->radius = 4;
    // particles.push_back(smallBall);
    
    // Particle* bigBall= new Particle(200, 100, 3.0);
    // bigBall->radius = 12;
    // particles.push_back(bigBall);
    
    Particle* particle = new Particle(Graphics::Width() / 2, Graphics::Height() / 2, 3.0);
    particle->radius = 15;
    particles.push_back(particle);
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
                mouseCursor.x = event.motion.x;
                mouseCursor.y = event.motion.y;
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (!leftMouseButtonDown && event.button.button == SDL_BUTTON_LEFT) {
                    leftMouseButtonDown = true;
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    mouseCursor.x = x;
                    mouseCursor.y = y;
                }
                break;
            case SDL_MOUSEBUTTONUP:
                if (leftMouseButtonDown && event.button.button == SDL_BUTTON_LEFT) {
                    leftMouseButtonDown = false;
                    Vec2 impulseDirection = (particles[0]->position - mouseCursor).UnitVector();
                    float impulseMagnitude = (particles[0]->position -mouseCursor).Magnitude() * 5.0;
                    particles[0]->velocity = impulseDirection * impulseMagnitude;
                }
                break;
        }
    }
}

// Update function (called each frame)
void Application::Update() {
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

    // apply forces to the particles  
    for (auto particle: particles) {
        particle->AddForce(pushForce);

        // apply the friction force 
        Vec2 friction = Force::GenerateFrictionForce(*particle, 10.0 * PIXELS_PER_METER);
        particle->AddForce(friction);
    }
   
    // integrate the acceleration and the velocity to find the new position
    for (auto particle: particles) {
        particle->Integrate(deltaTime);
    }
    
    // check the boundary of the window
    for (auto particle: particles) {
        if (particle->position.x - particle->radius <= 0) {
            particle->position.x = particle->radius;
            particle->velocity.x *= -0.9;
        } else if (particle->position.x + particle->radius >= Graphics::Width()) {
            particle->position.x = Graphics::Width() - particle->radius;
            particle->velocity.x *= - 0.9;
        }
        if (particle->position.y - particle->radius <= 0) {
            particle->position.y = particle->radius;
            particle->velocity.y *= -0.9;
        } else if (particle->position.y + particle->radius >= Graphics::Height()) {
            particle->position.y = Graphics::Height() - particle->radius;
            particle->velocity.y *= -0.9;
        }
    }
}

// Render function (called each frame)
void Application::Render() {
    Graphics::ClearScreen(0xFF1E4002);
    
    if (leftMouseButtonDown) {
        Graphics::DrawLine(particles[0]->position.x, particles[0]->position.y, mouseCursor.x, mouseCursor.y, 0xFF0000FF);
    }

    for (auto particle: particles) {
        Graphics::DrawFillCircle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);
    }
    Graphics::RenderFrame();
}

// Destroy function to delete objects and close the window
void Application::Destroy() {
    for (auto particle: particles) {
        delete particle;
    }
    Graphics::CloseWindow();
}
