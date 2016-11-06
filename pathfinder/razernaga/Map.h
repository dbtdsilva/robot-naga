//
// Created by myrddin on 06/11/16.
//

#ifndef RAZERNAGA_MAP_H
#define RAZERNAGA_MAP_H

#include <vector>
#include <SDL2/SDL.h>

class Map {
public:
    Map();
    Map(int cols, int rows, int square_precision, int square_size);
    ~Map();
private:
    bool create_window();
    void render();

    typedef struct PositionStats {
        PositionStats() : wall_counter(0), ground_counter(0), visited(0) { }
        unsigned int wall_counter;
        unsigned int ground_counter;
        unsigned int visited;
    } Stats;

    const int square_size_, square_precision_, rows_, cols_;
    std::vector<std::vector<Stats>> map_;

    SDL_Window* window;
    SDL_Renderer* renderer;
};


#endif //RAZERNAGA_MAP_H
