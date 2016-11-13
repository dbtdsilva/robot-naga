#include "MapAStar.h"
#include <cmath>
#include <algorithm>
#include "Map.h"

using namespace std;

MapAStar::MapAStar(Map *map) : map_(map), evaluation_function_(evaluation_function_default) {
    set_heuristic_function(MapAStar::heuristic_function_default);
}

void MapAStar::set_heuristic_function(
        function<double(const tuple<int,int>& p1, const tuple<int, int>& p2)> h_func) {
    heuristic_func_ = h_func;
}

double MapAStar::heuristic_function_default(const tuple<int,int>& p1, const tuple<int, int>& p2) {
    return sqrt(pow(get<0>(p2) - get<0>(p1), 2) + pow(get<1>(p2) - get<1>(p1), 2));
}

bool MapAStar::evaluation_function_default(const AStarNode* n1, const AStarNode* n2) {
    return (n1->cost + n1->heuristic) < (n2->cost + n2->heuristic);
}

vector<tuple<int, int>> MapAStar::discover_path(tuple<int,int> start, tuple<int, int> end) {
    vector<tuple<int, int>> final_path;
    // start and end position must be a wall or an unknown place
    if (map_->get_position_state(get<0>(start), get<1>(start)) == WALL ||
            map_->get_position_state(get<0>(end), get<1>(end)) == WALL)
        return final_path;
    // Nodes might be in more than one path (shared)
    vector<unique_ptr<AStarNode>> nodes_created_ownership;
    vector<AStarNode*> open_nodes;
    unique_ptr<AStarNode> start_node = make_unique<AStarNode>(start, nullptr, heuristic_func_(start, end), 0);

    open_nodes.push_back(start_node.get());
    nodes_created_ownership.push_back(std::move(start_node));

    vector<tuple<int,int>> visited;
    AStarNode *curr, *begin;
    while (!open_nodes.empty()) {
        // Get a new node to visit
        curr = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        if (curr->position == end) {
            while (curr != nullptr) {
                final_path.push_back(curr->position);
                curr = curr->parent;
            }
            return final_path;
        }
        // Skip if node was already visited
        if (std::find(visited.begin(), visited.end(), curr->position) != visited.end())
            continue;
        // Mark it as visited, prevents path cycles
        visited.push_back(curr->position);

        vector<tuple<int,int>> ramification_list;
        // List of possible value from the current one
        ramification_list.push_back(tuple<int,int>(get<0>(curr->position), get<1>(curr->position) + 1));
        ramification_list.push_back(tuple<int,int>(get<0>(curr->position) + 1, get<1>(curr->position)));
        ramification_list.push_back(tuple<int,int>(get<0>(curr->position) - 1, get<1>(curr->position)));
        ramification_list.push_back(tuple<int,int>(get<0>(curr->position), get<1>(curr->position) - 1));

        while (!ramification_list.empty()) {
            // Draw a new posibility node
            tuple<int,int> current_possibility = ramification_list.front();
            ramification_list.erase(ramification_list.begin());
            // Check if the new possibility is valid or not
            PositionState state = map_->get_position_state(get<0>(current_possibility), get<1>(current_possibility));
            if (state == UNKNOWN || state == GROUND) {
                begin = curr;
                // Check if that node was already on path from the solution or not, if yes, skip it.
                while (begin != nullptr) {
                    if (begin->position == current_possibility) break;
                    begin = begin->parent;
                }
                if (begin == nullptr) {
                    unique_ptr<AStarNode> n = make_unique<AStarNode>(current_possibility, curr,
                                                 heuristic_func_(current_possibility, end), curr->cost + 1);
                    open_nodes.push_back(n.get());
                    nodes_created_ownership.push_back(std::move(n));
                }
            }
        }
        // Sort the new ramifications of nodes found by their evaluation value
        std::sort(open_nodes.begin(), open_nodes.end(), evaluation_function_default);
    }
    // No path was found
    return final_path;
}