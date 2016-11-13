#ifndef RAZERNAGA_MAPASTAR_H
#define RAZERNAGA_MAPASTAR_H

#include <tuple>
#include <functional>
#include <memory>
#include <vector>

// Forwarded Declared
//#include "Map.h"
class Map;

class MapAStar {
public:
    MapAStar(Map *map);
    std::vector<std::tuple<int, int>> discover_path(std::tuple<int,int> start, std::tuple<int, int> end);
    void set_heuristic_function(std::function<double(const std::tuple<int,int>&, const std::tuple<int, int>&)> h_func);

private:
    typedef struct Node {
        Node(std::tuple<int, int> position, Node* parent, double heuristic, double cost) :
                position(position), parent(parent), heuristic(heuristic), cost(cost) { };
        double heuristic, cost;
        Node* parent;
        std::tuple<int, int> position;
    } AStarNode;

    static bool compare(const AStarNode* n1, const AStarNode* n2);
    static bool evaluation_function_default(const std::shared_ptr<AStarNode>&, const std::shared_ptr<AStarNode>&);
    static double heuristic_function_default(const std::tuple<int,int>& p1, const std::tuple<int, int>& p2);

    Map *map_;
    std::function<double(const std::tuple<int,int>&, const std::tuple<int, int>&)> heuristic_func_;
    std::function<bool(const std::shared_ptr<AStarNode>&, const std::shared_ptr<AStarNode>&)> evaluation_function_;
};

#endif //RAZERNAGA_MAPASTAR_H
