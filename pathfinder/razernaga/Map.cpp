//
// Created by myrddin on 06/11/16.
//

#include "Map.h"
#include <iostream>

using namespace std;

Map::Map() : Map(14, 7, 8) {
}

Map::Map(int cols, int rows, int square_precision) :
        map_(cols * square_precision * 2, vector<Stats>(rows * square_precision * 2, Stats())),
        cols_(cols), rows_(rows), square_precision_(square_precision) {
}

Map::~Map() {
}

void Map::enable_debug() {
    if (map_debug_ == nullptr)
        map_debug_ = make_unique<MapSDL2>(cols_, rows_, square_precision_, 4);
}

bool Map::increase_wall_counter(const double& x, const double& y) {
    int new_x = (int) round(x * (square_precision_ / 2.0) + cols_ * square_precision_);
    int new_y = (int) round(-y * (square_precision_ / 2.0) + rows_ * square_precision_);
    if (!validate_position(new_x, new_y)) return false;
    map_[new_x][new_y].wall_counter++;

    evaluate_position(new_x, new_y);
    return true;
}
bool Map::increase_ground_counter(const double& x, const double& y) {
    int new_x = (int) round(x * (square_precision_ / 2.0) + cols_ * square_precision_);
    int new_y = (int) round(-y * (square_precision_ / 2.0) + rows_ * square_precision_);
    if (!validate_position(new_x, new_y)) return false;
    map_[new_x][new_y].ground_counter++;

    evaluate_position(new_x, new_y);
    return true;
}
bool Map::increase_visited_counter(const double& x, const double& y) {
    int new_x = (int) round(x * (square_precision_ / 2.0) + cols_ * square_precision_);
    int new_y = (int) round(-y * (square_precision_ / 2.0) + rows_ * square_precision_);
    if (!validate_position(new_x, new_y)) return false;
    map_[new_x][new_y].visited++;

    evaluate_position(new_x, new_y);
    return true;
}

void Map::evaluate_position(const int& x, const int& y) {
    if (map_[x][y].visited > 0)
        map_[x][y].is_ground = true;
    else
        map_[x][y].is_ground = map_[x][y].wall_counter <= map_[x][y].ground_counter * 0.2;

    if (map_debug_ != nullptr) {
        if (map_[x][y].is_ground)
            map_debug_->set_color(x, y, 0, 255, 0, 255);
        else
            map_debug_->set_color(x, y, 0, 0, 0, 255);
    }

}

bool Map::validate_position(const int& x, const int& y) {
    return (x >= 0 && x < cols_ * square_precision_ * 2 &&
            y >= 0 && y < rows_ * square_precision_ * 2);
}

