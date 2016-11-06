//
// Created by myrddin on 06/11/16.
//

#include "Map.h"
#include <iostream>

using namespace std;

Map::Map() : Map(14, 7, 4, 8) {
}

Map::Map(int cols, int rows, int square_precision, int square_size) :
        map_(cols * square_precision * 2, vector<Stats>(rows * square_precision * 2, Stats())),
        square_size_(square_size), cols_(cols), rows_(rows), square_precision_(square_precision) {
    cout << map_.size() << " " << map_[0].size() << endl;

    create_window();
}

Map::~Map() {
    if (window != nullptr) {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
}

void Map::render() {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    SDL_Rect rect;
    int middle_x = (int)map_.size() / 2;
    int middle_y = (int)map_[0].size() / 2;
    for (int x = 0; x < map_.size(); x++) {
        for (int y = 0; y < map_[x].size(); y++) {
            rect.x = x * (square_size_ + 1);
            rect.y = y * (square_size_ + 1);
            rect.w = square_size_;
            rect.h = square_size_;
            if (middle_x == x && middle_y == y) {
                SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
                SDL_RenderFillRect(renderer, &rect);
                //SDL_RenderDrawRect(renderer, &rect);
            } else {
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_RenderFillRect(renderer, &rect);
                //SDL_RenderDrawRect(renderer, &rect);
            }
        }
    }
    SDL_RenderPresent(renderer);
}

bool Map::create_window() {
    if (SDL_Init(SDL_INIT_EVERYTHING)) {
        return false;
    }
    window = SDL_CreateWindow("RazerNaga Vision", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              cols_ * square_precision_ * 2 * (square_size_ + 1),
                              rows_ * square_precision_ * 2 * (square_size_ + 1),
        SDL_WINDOW_OPENGL);
    if (window == nullptr) {
        return false;
    }
    renderer = SDL_CreateRenderer(window, -1, 0);
    if (renderer == nullptr) {
        return false;
    }

    render();
    return true;
}