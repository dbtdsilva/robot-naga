#ifndef RAZERNAGA_MAP_H
#define RAZERNAGA_MAP_H

#include <vector>
#include <memory>
#include <SDL2/SDL.h>
#include "MapSDL2.h"
#include "MapAStar.h"

typedef enum { GROUND, WALL, UNKNOWN } PositionState;

class Map {
public:
    Map();
    Map(int cols, int rows, int square_precision);
    ~Map();

    bool increase_wall_counter(const double& x, const double& y);
    bool increase_ground_counter(const double& x, const double& y);
    bool increase_visited_counter(const double& x, const double& y);
    PositionState get_position_state(const int& x, const int& y) const;
    void enable_debug();
    void render_map();
    std::tuple<int, int> get_map_dimensions() const;
private:
    void evaluate_position(const int& x, const int& y);
    bool validate_position(const int& x, const int& y);

    typedef struct PositionStats {
        PositionStats() : wall_counter(0), ground_counter(0), visited(0), state(UNKNOWN) { }
        unsigned int wall_counter;
        unsigned int ground_counter;
        unsigned int visited;
        PositionState state;
    } Stats;

    const int square_precision_, rows_, cols_;
    std::vector<std::vector<Stats>> map_;
    std::unique_ptr<MapSDL2> map_debug_;
    std::tuple<int, int> last_visited_pos_;
    std::unique_ptr<MapAStar> path_algorithm_;
};


#endif //RAZERNAGA_MAP_H
