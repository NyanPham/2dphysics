#include "Application.h"

bool Application::IsRunning() {
    return running;
}

// Setup function (executed once in the beginning)
void Application::Setup() {
    running = Graphics::OpenWindow();
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

}

// Render function (called each frame)
void Application::Render() {
    Graphics::ClearScreen(0xFF056263);
    Graphics::DrawFillCircle(200, 200, 40, 0xFFFFFFFF);
    Graphics::RenderFrame();
}

// Destroy function to delete objects and close the window
void Application::Destroy() {
    Graphics::CloseWindow();
}
