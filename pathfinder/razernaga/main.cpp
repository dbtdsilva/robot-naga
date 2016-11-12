#include <iostream>
#include "RazerNaga.h"
#include <SDL2/SDL.h>

using namespace std;

int main(int argc, char** argv)
{
    string host = "localhost";
    int rob_id = 0;
    cout << "Compiled with Qt Version " << QT_VERSION_STR << endl;
    cout << " RazerNaga Robot " << endl << " Copyright 2002-2016 Universidade de Aveiro" << endl;

    while (argc > 2)
    {
        if (strcmp(argv[1], "-host") == 0)
        {
            host = string(argv[2]);
        }
        else if (strcmp(argv[1], "-pos") == 0)
        {
            if (sscanf(argv[2], "%d", &rob_id) != 1)
                argc = 0;
        }
        else
        {
            break;
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        cerr << "Bad number of parameters" << endl <<
                "SYNOPSIS: GUISample [-host hostname] [-robname robotname] [-pos posnumber]" << endl;
        return 1;
    }

    RazerNaga app(argc, argv, rob_id, host);
    return app.exec();

    /*SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    SDL_Event event;
    SDL_Rect r;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't initialize SDL: %s", SDL_GetError());
        return 3;
    }

    window = SDL_CreateWindow("SDL_CreateTexture",
                              SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED,
                              1024, 768,
                              SDL_WINDOW_RESIZABLE);

    r.w = 50;
    r.h = 50;

    renderer = SDL_CreateRenderer(window, -1, 0);

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 1024, 768);

    while (1) {
        SDL_PollEvent(&event);
        if(event.type == SDL_QUIT)
            break;
        r.x = rand()%500;
        r.y = rand()%500;

        SDL_SetRenderTarget(renderer, texture);
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderFillRect(renderer, &r);

        SDL_SetRenderTarget(renderer, NULL);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
    }
    SDL_DestroyRenderer(renderer);
    SDL_Quit();
    return 0;*/
}
