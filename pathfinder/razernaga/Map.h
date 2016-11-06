//
// Created by myrddin on 06/11/16.
//

#ifndef RAZERNAGA_MAP_H
#define RAZERNAGA_MAP_H

#include <vector>
#include <memory>
#include <SDL2/SDL.h>
#include "MapSDL2.h"

class Map {
public:
    Map();
    Map(int cols, int rows, int square_precision);
    ~Map();

    void enable_debug();
private:

    typedef struct PositionStats {
        PositionStats() : wall_counter(0), ground_counter(0), visited(0) { }
        unsigned int wall_counter;
        unsigned int ground_counter;
        unsigned int visited;
    } Stats;

    const int square_precision_, rows_, cols_;
    std::vector<std::vector<Stats>> map_;
    std::unique_ptr<MapSDL2> map_debug_;
};


#endif //RAZERNAGA_MAP_H
