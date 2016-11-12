//
// Created by myrddin on 12/11/16.
//

#include "MapAStar.h"
#include <algorithm>
using namespace std;

MapAStar::MapAStar(const Map *map) : map_(map), evaluation_function_(evaluation_function_default) {
    set_heuristic_function(MapAStar::heuristic_function_default);
}


void MapAStar::set_heuristic_function(
        std::function<double(const std::tuple<int,int>& p1, const std::tuple<int, int>& p2)> h_func) {
    heuristic_func_ = h_func;
}

double MapAStar::heuristic_function_default(const std::tuple<int,int>& p1, const std::tuple<int, int>& p2) {
    return sqrt(pow(get<0>(p2) - get<0>(p1), 2) + pow(get<1>(p2) - get<1>(p1), 2));
}

bool MapAStar::evaluation_function_default(AStarNode* n1, AStarNode *n2) {
    return (n1->cost + n1->heuristic) < (n2->cost + n2->heuristic);
}

AStarNode* MapAStar::discover_path(std::tuple<int,int> start, std::tuple<int, int> end) {
    std::vector<AStarNode* > open_nodes;

    AStarNode* start_node = new AStarNode(start, nullptr, 0, heuristic_func_(start, end));
    open_nodes.push_back(start_node);

    AStarNode *curr, *begin;
    vector<tuple<int,int>> visited;

    while (!open_nodes.empty()) {
        // Get a new node to visit
        curr = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        if (curr->position == end)
            return curr;
        // Skip if node was already visited
        if (std::find(visited.begin(), visited.end(), curr->position) != visited.end())
            continue;
        // Mark it as visited, prevents path cycles
        visited.push_back(curr->position);

        vector<tuple<int,int>> lnewlist;
        // List of possible value from the current one
        lnewlist.push_back(tuple<int,int>(get<0>(curr->position), get<1>(curr->position) + 1));
        lnewlist.push_back(tuple<int,int>(get<0>(curr->position) + 1, get<1>(curr->position)));
        lnewlist.push_back(tuple<int,int>(get<0>(curr->position) - 1, get<1>(curr->position)));
        lnewlist.push_back(tuple<int,int>(get<0>(curr->position), get<1>(curr->position) - 1));

        while (!lnewlist.empty()) {
            // Draw a new posibility node
            tuple<int,int> current_possibility = lnewlist.front();
            lnewlist.erase(lnewlist.begin());

            // Check if the new possibility is valid or not
            if (!map_->is_wall(get<0>(current_possibility), get<1>(current_possibility))) {
                begin = curr;
                // Check if that node was already on path from the solution or not, if yes, skip it.
                while (begin != nullptr) {
                    if (begin->position == current_possibility) break;
                    begin = begin->parent;
                }
                if (begin == nullptr)
                    open_nodes.push_back(new AStarNode(current_possibility, curr, curr->cost + 1,
                                                       heuristic_func_(current_possibility, end)));
            }
        }
        // Sort the new ramifications of nodes found by their evaluation value
        std::sort(open_nodes.begin(), open_nodes.end(), evaluation_function_);
    }
    // No path was found
    return nullptr;
}