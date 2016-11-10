//
// Created by myrddin on 06/11/16.
//

#include "MapSDL2.h"
#include <iostream>

using namespace std;

MapSDL2::MapSDL2(int cols, int rows, int square_precision, int square_size) :
        cols_(cols), rows_(rows), square_precision_(square_precision), square_size_(square_size),
        map_(cols * square_precision * 2, vector<Color>(rows * square_precision * 2, Color())){
    if (SDL_Init(SDL_INIT_EVERYTHING)) return;
    window = SDL_CreateWindow("RazerNaga Vision", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              cols_ * square_precision_ * 2 * (square_size_ + 1),
                              rows_ * square_precision_ * 2 * (square_size_ + 1),
                              SDL_WINDOW_OPENGL);
    if (window == nullptr) return;
    renderer = SDL_CreateRenderer(window, -1, 0);
    if (renderer == nullptr) return;
    render_full_map();
}

MapSDL2::~MapSDL2() {
    if (window != nullptr) {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
}

int requests = 0;
void MapSDL2::set_color(int x, int y, Uint8 R, Uint8 G, Uint8 B, Uint8 A) {
    if (map_[x][y].R != R || map_[x][y].G != G || map_[x][y].B != B || map_[x][y].A != A)
    {
        map_[x][y].R = R;
        map_[x][y].G = G;
        map_[x][y].B = B;
        map_[x][y].A = A;
        if (requests++ % 1000 == 0)
            render_full_map();
    }
}

void MapSDL2::render_full_map() {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    SDL_Rect rect;
    for (int x = 0; x < map_.size(); x++) {
        for (int y = 0; y < map_[x].size(); y++) {
            rect.x = x * (square_size_ + 1);
            rect.y = y * (square_size_ + 1);
            rect.w = square_size_;
            rect.h = square_size_;

            SDL_SetRenderDrawColor(renderer, map_[x][y].R, map_[x][y].G, map_[x][y].B, map_[x][y].A);
            SDL_RenderFillRect(renderer, &rect);
        }
    }
    SDL_RenderPresent(renderer);

}