#include "Map.h"
#include <iostream>

using namespace std;

Map::Map() : Map(14, 7, 8) {
}

Map::Map(int cols, int rows, int square_precision) :
        map_(cols * square_precision * 2, vector<Stats>(rows * square_precision * 2, Stats())),
        cols_(cols), rows_(rows), square_precision_(square_precision), last_visited_pos_(0,0),
        path_algorithm_(make_unique<MapAStar>(this)) {
}

Map::~Map() {
}

void Map::enable_debug() {
    if (map_debug_ == nullptr)
        map_debug_ = make_unique<MapSDL2>(cols_, rows_, square_precision_, 4);
}

std::tuple<int, int> Map::get_map_dimensions() const{
    return tuple<int, int>(map_.size(), map_[0].size());
}

PositionState Map::get_position_state(const int& x, const int& y) const {
    return map_[x][y].state;
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

    get<0>(last_visited_pos_) = new_x;
    get<1>(last_visited_pos_) = new_y;
    evaluate_position(new_x, new_y);
    return true;
}

void Map::evaluate_position(const int& x, const int& y) {
    if (map_[x][y].visited > 0)
        map_[x][y].state = GROUND;
    else
        map_[x][y].state = map_[x][y].wall_counter <= map_[x][y].ground_counter * 0.1 ? GROUND : WALL;

    if (map_debug_ != nullptr) {
        if (map_[x][y].state == GROUND)
            map_debug_->set_color(x, y, 0, 255, 0, 255);
        else
            map_debug_->set_color(x, y, 0, 0, 0, 255);
    }

}

void Map::render_map() {
    cout << "Rendering" << endl;
    if (map_debug_ == nullptr)
        return;

    vector<tuple<int, int, Uint8, Uint8, Uint8, Uint8>> temporary_paintings;
    std::vector<int> color;
    // Paint the path to the objective location
    AStarNode* node = path_algorithm_->discover_path(
            last_visited_pos_, tuple<int, int>(get<0>(last_visited_pos_) + 10, get<1>(last_visited_pos_) + 10));
    while (node != nullptr) {
        color = map_debug_->get_color(get<0>(node->position), get<1>(node->position));
        temporary_paintings.push_back(tuple<int, int, Uint8, Uint8, Uint8, Uint8>(
                get<0>(node->position), get<1>(node->position), color[0], color[1], color[2], color[3]));
        map_debug_->set_color(get<0>(node->position), get<1>(node->position), 255, 0, 0, 255);
        node = node->parent;
    }
    // Paint the current position
    color = map_debug_->get_color(get<0>(last_visited_pos_), get<1>(last_visited_pos_));
    map_debug_->set_color(get<0>(last_visited_pos_), get<1>(last_visited_pos_), 0, 0, 255, 255);
    temporary_paintings.push_back(tuple<int, int, Uint8, Uint8, Uint8, Uint8>(
            get<0>(last_visited_pos_), get<1>(last_visited_pos_), color[0], color[1], color[2], color[3]));
    // Render the map
    map_debug_->render_full_map();

    // Give map the previous color before entering the current position and A*
    while (!temporary_paintings.empty()) {
        tuple<int, int, Uint8, Uint8, Uint8, Uint8> value = temporary_paintings.back();
        temporary_paintings.pop_back();
        map_debug_->set_color(get<0>(value), get<1>(value), get<2>(value), get<3>(value), get<4>(value), get<5>(value));
    }
    cout << "Finished" << endl;
}

bool Map::validate_position(const int& x, const int& y) {
    return (x >= 0 && x < cols_ * square_precision_ * 2 &&
            y >= 0 && y < rows_ * square_precision_ * 2);
}

