#ifndef APPLICATION_H
#define APPLICATION_H

#include "./Graphics.h"
#include "./Physics/World.h"

class Application {
    private: 
        bool running = false;
        bool debug = false;

        World* world;
        SDL_Texture* bgTexture;

    public:
        Application() = default;
        ~Application() = default;
        bool IsRunning();
        void Setup();
        void Input();
        void Update();
        void Render();
        void Destroy();
};

#endif
