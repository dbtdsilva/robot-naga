#ifndef RAZERNAGA_MAP_H
#define RAZERNAGA_MAP_H

#include <vector>
#include <memory>
#include <SDL2/SDL.h>
#include "MapSDL2.h"
#include "MapAlgorithms.h"

#define M_X(tuple)  std::get<0>(tuple)
#define M_Y(tuple)  std::get<1>(tuple)

typedef enum { GROUND, WALL, UNKNOWN } PositionState;

class Map {
public:
    Map();
    Map(int cols, int rows, int square_precision);

    bool increase_wall_counter(const double& x, const double& y);
    bool increase_ground_counter(const double& x, const double& y);
    bool increase_ground_counter_range(const double& sx, const double& sy, const double& fx, const double& fy);
    bool increase_visited_counter(const double& x, const double& y);

    void set_target_nearest_exit();
    void set_target_starter_area();
    std::vector<std::tuple<double, double>>& get_calculated_path();
    PositionState get_position_state(const int& x, const int& y) const;

    void enable_debug();
    void render_map();
private:
    std::tuple<int, int> convert_to_map_coordinates(const std::tuple<double, double>&);
    std::tuple<int, int> convert_to_map_coordinates(const double&, const double&);
    std::tuple<double, double> convert_from_map_coordinates(const int&, const int&);
    std::tuple<double, double> convert_from_map_coordinates(const std::tuple<int, int>&);
    void evaluate_position(const int& x, const int& y);

    bool validate_position(const int& x, const int& y) const;
    bool validate_position(const std::tuple<int, int>&) const;

    typedef struct PositionStats {
        PositionStats() : referred(false), wall_counter(0), ground_counter(0), visited(0), state(UNKNOWN) { }
        bool referred;
        int wall_counter, ground_counter;
        unsigned int visited;
        PositionState state;
    } Stats;

    const int square_precision_, rows_, cols_;
    std::vector<std::vector<Stats>> map_;
    std::unique_ptr<MapSDL2> map_debug_;
    std::unique_ptr<MapAlgorithms> path_algorithm_;
    std::tuple<int, int> last_visited_pos_;
    std::vector<std::tuple<int, int>> calculated_target_path_;
    std::vector<std::tuple<double, double>> calculated_target_path_converted_;
};


#endif //RAZERNAGA_MAP_H
