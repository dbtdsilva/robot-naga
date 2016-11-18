#include "Map.h"
#include <iostream>
#include <algorithm>

using namespace std;

Map::Map() : Map(14, 7, 8) {
}

Map::Map(int cols, int rows, int square_precision) :
        map_(cols * square_precision * 2, vector<Stats>(rows * square_precision * 2, Stats())),
        cols_(cols), rows_(rows), square_precision_(square_precision), last_visited_pos_(convert_to_map_coordinates(0,0)),
        path_algorithm_(make_unique<MapAlgorithms>(this)), ptr_objective_(nullptr) {
    srand(time(NULL));
}

void Map::enable_debug() {
    if (map_debug_ == nullptr)
        map_debug_ = make_unique<MapSDL2>(cols_, rows_, square_precision_, 4);
}

PositionState Map::get_position_state(const int& x, const int& y) const {
    return map_[x][y].state;
}

std::tuple<int, int> Map::convert_to_map_coordinates(const std::tuple<double, double>& real_coordinates) {
    return convert_to_map_coordinates(M_X(real_coordinates), M_Y(real_coordinates));
}

std::tuple<int, int> Map::convert_to_map_coordinates(const double& x, const double& y) {
    return tuple<int, int>(static_cast<int>(round(x * (square_precision_ / 2.0) + cols_ * square_precision_)),
                           static_cast<int>(round(-y * (square_precision_ / 2.0) + rows_ * square_precision_)));
}

std::tuple<double, double> Map::convert_from_map_coordinates(const int& x, const int& y) {
    return tuple<double, double>(2.0 * (static_cast<double>(x) / square_precision_ - cols_),
                                 -2.0 * (static_cast<double>(y) / square_precision_ - rows_));
}

std::tuple<double, double> Map::convert_from_map_coordinates(const std::tuple<int, int>& map_coordinates) {
    return convert_from_map_coordinates(M_X(map_coordinates), M_Y(map_coordinates));
}

vector<tuple<double, double>> Map::convert_trajectory_to_discrete(const vector<tuple<int, int>>& trajectory) {
    vector<tuple<double, double>> discrete_trajectory;
    for (const tuple<int, int>& point_map : trajectory) {
        tuple<double,double> point = convert_from_map_coordinates(point_map);
        tuple<int,int> new_value = tuple<int, int>(
                floor((M_X(point) + 1.0) / 2.0) * 2.0,
                floor((M_Y(point) + 1.0) / 2.0) * 2.0);
        if (std::find(discrete_trajectory.begin(), discrete_trajectory.end(), new_value) == discrete_trajectory.end()) {
            discrete_trajectory.push_back(new_value);
            //printf("%2d %2d %4.2f %4.2f\n", M_X(new_value), M_Y(new_value), M_X(point), M_Y(point) );
        }
    }
    return discrete_trajectory;
}

void Map::set_target_nearest_exit() {
    calculated_target_path_ = path_algorithm_->flood_fill(last_visited_pos_, 8);
    calculated_target_path_converted_.clear();
    calculated_target_path_converted_ = convert_trajectory_to_discrete(calculated_target_path_);

}
void Map::set_target_starter_area() {
    calculated_target_path_ = path_algorithm_->astar_shortest_path(
            last_visited_pos_, convert_to_map_coordinates(tuple<int, int>(0, 0)));
    calculated_target_path_converted_.clear();
    calculated_target_path_converted_ = convert_trajectory_to_discrete(calculated_target_path_);
}

void Map::set_objective(const std::tuple<double, double>& objective) {
    ptr_objective_ = make_unique<std::tuple<int,int>>(convert_to_map_coordinates(objective));
}

vector<tuple<double,double>>& Map::get_calculated_path() {
    return calculated_target_path_converted_;
}

bool Map::increase_wall_counter(const double& x, const double& y) {
    tuple<int, int> position = convert_to_map_coordinates(x, y);
    if (!validate_position(position)) return false;
    map_[M_X(position)][M_Y(position)].wall_counter++;
    evaluate_position(M_X(position), M_Y(position));
    return true;
}

bool Map::increase_ground_counter(const double& x, const double& y) {
    tuple<int, int> position = convert_to_map_coordinates(x, y);
    if (!validate_position(position)) return false;
    map_[M_X(position)][M_Y(position)].ground_counter++;
    evaluate_position(M_X(position), M_Y(position));
    return true;
}

bool Map::increase_ground_counter_range(const double& sx, const double& sy, const double& fx, const double& fy) {
    long double dx, dy;
    const int N_POINTS = 100;

    dx = (fx - sx) / N_POINTS;
    dy = (fy - sy) / N_POINTS;

    vector<Stats*> modified_points;
    for (int points = 0; points < N_POINTS; points++) {
        tuple<int, int> position = convert_to_map_coordinates(sx + points * dx, sy + points * dy);
        if (map_[M_X(position)][M_Y(position)].referred)
            continue;
        map_[M_X(position)][M_Y(position)].referred = true;
        modified_points.push_back(&map_[M_X(position)][M_Y(position)]);
        increase_ground_counter(sx + points * dx, sy + points * dy);
    }

    for (Stats* element : modified_points)
        element->referred = false;
}

bool Map::increase_visited_counter(const double& x, const double& y) {
    tuple<int, int> position = convert_to_map_coordinates(x, y);
    if (!validate_position(position)) return false;
    map_[M_X(position)][M_Y(position)].visited++;

    M_X(last_visited_pos_) = M_X(position);
    M_Y(last_visited_pos_) = M_Y(position);
    evaluate_position(M_X(position), M_Y(position));
    return true;
}

void Map::evaluate_position(const int& x, const int& y) {
    if (map_[x][y].visited > 0)
        map_[x][y].state = GROUND;
    else {
        map_[x][y].state = map_[x][y].wall_counter <= map_[x][y].ground_counter * 0.15 ? GROUND : WALL;
    }

    if (map_debug_ != nullptr) {
        if (map_[x][y].state == GROUND)
            map_debug_->set_color(x, y, 0, 255, 0, 255);
        else if (map_[x][y].state == WALL)
            map_debug_->set_color(x, y, 0, 0, 0, 255);
    }
}

void Map::render_map() {
    if (map_debug_ == nullptr)
        return;

    if (map_debug_->exit_requested()) {
        cout << "RazerNaga has been requested to exit!" << endl;
        std::exit(0);
    }

    vector<tuple<int, int, Uint8, Uint8, Uint8, Uint8>> temporary_paintings;
    std::vector<int> color;
    // Paint the path to the objective location
    for (auto path_node : calculated_target_path_) {
        color = map_debug_->get_color(M_X(path_node), M_Y(path_node));
        temporary_paintings.push_back(tuple<int, int, Uint8, Uint8, Uint8, Uint8>(
                M_X(path_node), M_Y(path_node), color[0], color[1], color[2], color[3]));
        map_debug_->set_color(M_X(path_node), M_Y(path_node), 255, 0, 0, 255);
    }
    // Objective representation
    if (ptr_objective_ != nullptr) {
        color = map_debug_->get_color(M_X(*ptr_objective_), M_Y(*ptr_objective_));
        map_debug_->set_color(M_X(*ptr_objective_), M_Y(*ptr_objective_), 0, 0, 204, 255);
        temporary_paintings.push_back(tuple<int, int, Uint8, Uint8, Uint8, Uint8>(
                M_X(*ptr_objective_), M_Y(*ptr_objective_), color[0], color[1], color[2], color[3]));
    }
    // Paint the current position
    color = map_debug_->get_color(M_X(last_visited_pos_), M_Y(last_visited_pos_));
    map_debug_->set_color(M_X(last_visited_pos_), M_Y(last_visited_pos_), 0, 0, 255, 255);
    temporary_paintings.push_back(tuple<int, int, Uint8, Uint8, Uint8, Uint8>(
            M_X(last_visited_pos_), M_Y(last_visited_pos_), color[0], color[1], color[2], color[3]));

    // Render the map
    map_debug_->render_full_map();

    // Give map the previous color before entering the current position and A*
    while (!temporary_paintings.empty()) {
        auto value = temporary_paintings.back();
        temporary_paintings.pop_back();
        map_debug_->set_color(get<0>(value), get<1>(value), get<2>(value), get<3>(value), get<4>(value), get<5>(value));
    }
}

bool Map::validate_position(const int& x, const int& y) const {
    return (x >= 0 && x < cols_ * square_precision_ * 2 &&
            y >= 0 && y < rows_ * square_precision_ * 2);
}

bool Map::validate_position(const tuple<int, int>& value) const {
    return validate_position(M_X(value), M_Y(value));
}

