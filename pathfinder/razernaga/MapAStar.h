#ifndef RAZERNAGA_MAPASTAR_H
#define RAZERNAGA_MAPASTAR_H

#include <tuple>
#include <functional>

// Forwarded Declared
//#include "Map.h"
class Map;

typedef struct AStarNode {
    AStarNode(std::tuple<int, int> position, AStarNode* parent, double heuristic, double cost) :
            position(position), parent(parent), heuristic(heuristic), cost(cost) { }
    double heuristic, cost;
    AStarNode *parent;
    std::tuple<int, int> position;
} AStarNode;

class MapAStar {
public:
    MapAStar(Map *map);
    AStarNode* discover_path(std::tuple<int,int> start, std::tuple<int, int> end);
    void set_heuristic_function(std::function<double(const std::tuple<int,int>&, const std::tuple<int, int>&)> h_func);

private:
    static bool evaluation_function_default(AStarNode* n1, AStarNode *n2);
    static double heuristic_function_default(const std::tuple<int,int>& p1, const std::tuple<int, int>& p2);

    Map *map_;
    std::function<double(const std::tuple<int,int>&, const std::tuple<int, int>&)> heuristic_func_;
    std::function<bool(AStarNode*, AStarNode*)> evaluation_function_;
};


#endif //RAZERNAGA_MAPASTAR_H
