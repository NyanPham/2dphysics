#include "Application.h"
#include "./Physics/Constants.h"

bool Application::IsRunning() {
    return running;
}

// Setup function (executed once in the beginning)
void Application::Setup() {
    running = Graphics::OpenWindow();
    
    particle = new Particle(50, 100, 1.0);
    particle->radius = 4;
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

    // apply a "wind" force  
    Vec2 wind = Vec2(0.2 * PIXELS_PER_METER, 0.0);
    particle->AddForce(wind);

    // integrate the acceleration and the velocity to find the new position
    particle->Integrate(deltaTime);

    if (particle->position.x - particle->radius <= 0) {
        particle->position.x = particle->radius;
        particle->velocity.x *= -1.0;
    } else if (particle->position.x + particle->radius >= Graphics::Width()) {
        particle->position.x = Graphics::Width() - particle->radius;
        particle->velocity.x *= -1.0;
    }
    if (particle->position.y - particle->radius <= 0) {
        particle->position.y = particle->radius;
        particle->velocity.y *= -1.0;
    } else if (particle->position.y + particle->radius >= Graphics::Height()) {
        particle->position.y = Graphics::Height() - particle->radius;
        particle->velocity.y *= -1.0;
    }
}

// Render function (called each frame)
void Application::Render() {
    Graphics::ClearScreen(0xFF056263);
    Graphics::DrawFillCircle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);
    Graphics::RenderFrame();
}

// Destroy function to delete objects and close the window
void Application::Destroy() {
    delete particle;
    Graphics::CloseWindow();
}
