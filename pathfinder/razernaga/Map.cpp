//
// Created by myrddin on 06/11/16.
//

#include "Map.h"
#include <iostream>

using namespace std;

Map::Map() : Map(14, 7, 4) {
}

Map::Map(int cols, int rows, int square_precision) :
        map_(cols * square_precision * 2, vector<Stats>(rows * square_precision * 2, Stats())),
        cols_(cols), rows_(rows), square_precision_(square_precision) {
}

Map::~Map() {
}

void Map::enable_debug() {
    if (map_debug_ == nullptr)
        map_debug_ = make_unique<MapSDL2>(cols_, rows_, square_precision_, 8);
}