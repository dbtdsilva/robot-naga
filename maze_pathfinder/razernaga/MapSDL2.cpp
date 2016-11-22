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

    const int height = cols_ * square_precision_ * 2 * (square_size_ + 1);
    const int width = rows_ * square_precision_ * 2 * (square_size_ + 1);
    window_ = SDL_CreateWindow("RazerNaga Vision", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              height, width, SDL_WINDOW_OPENGL);
    if (window_ == nullptr) return;
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (renderer_ == nullptr) return;
    render_full_map();
}

MapSDL2::~MapSDL2() {
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
    }
    SDL_Quit();
}

bool MapSDL2::exit_requested() {
    while(SDL_PollEvent(&events_)) {
        if (events_.type == SDL_QUIT)
            return true;
    }
    return false;
}

void MapSDL2::set_color(int x, int y, Uint8 R, Uint8 G, Uint8 B, Uint8 A) {
    if (map_[x][y].R != R || map_[x][y].G != G || map_[x][y].B != B || map_[x][y].A != A)
    {
        map_[x][y].R = R;
        map_[x][y].G = G;
        map_[x][y].B = B;
        map_[x][y].A = A;
    }
}

std::vector<int> MapSDL2::get_color(const int& x, const int& y) const {
    return {map_[x][y].R, map_[x][y].G, map_[x][y].B, map_[x][y].A};
}

void MapSDL2::render_full_map() {
    SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
    SDL_RenderClear(renderer_);

    SDL_Rect rect;
    for (int x = 0; x < map_.size(); x++) {
        for (int y = 0; y < map_[x].size(); y++) {
            rect.x = x * (square_size_ + 1);
            rect.y = y * (square_size_ + 1);
            rect.w = square_size_;
            rect.h = square_size_;

            SDL_SetRenderDrawColor(renderer_, map_[x][y].R, map_[x][y].G, map_[x][y].B, map_[x][y].A);
            SDL_RenderFillRect(renderer_, &rect);
        }
    }
    SDL_RenderPresent(renderer_);

}