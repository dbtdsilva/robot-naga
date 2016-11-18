#include "MapAlgorithms.h"
#include <cmath>
#include <algorithm>
#include "Map.h"

using namespace std;

MapAlgorithms::MapAlgorithms(Map *map) : map_(map), evaluation_function_(evaluation_function_default) {
    set_heuristic_function(MapAlgorithms::heuristic_function_default);
}

void MapAlgorithms::set_heuristic_function(
        function<double(const tuple<int,int>& p1, const tuple<int, int>& p2)> h_func) {
    heuristic_func_ = h_func;
}

double MapAlgorithms::heuristic_function_default(const tuple<int,int>& p1, const tuple<int, int>& p2) {
    return sqrt(pow(M_X(p2) - M_X(p1), 2) + pow(M_Y(p2) - M_Y(p1), 2));
}

bool MapAlgorithms::evaluation_function_default(const Node* n1, const Node* n2) {
    return (n1->cost + n1->heuristic) < (n2->cost + n2->heuristic);
}

vector<tuple<int, int>> MapAlgorithms::flood_fill(const tuple<int, int>& start, const int& minimum_distance) {
    vector<tuple<int, int>> final_path;
    // start and end position must be a wall or an unknown place
    if (map_->get_position_state(M_X(start), M_Y(start)) == WALL)
        return final_path;
    // Nodes might be in more than one path (shared)
    vector<unique_ptr<Node>> nodes_created_ownership;
    vector<Node*> open_nodes;
    auto start_node = make_unique<Node>(start, nullptr, 0, 0);

    open_nodes.push_back(start_node.get());
    nodes_created_ownership.push_back(std::move(start_node));

    vector<tuple<int,int>> visited;
    Node *curr, *begin;
    while (!open_nodes.empty()) {
        // Get a new node to visit
        curr = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        if (map_->get_position_state(M_X(curr->position), M_Y(curr->position)) == UNKNOWN &&
                curr->cost >= minimum_distance) {
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
        ramification_list.push_back(tuple<int,int>(M_X(curr->position), M_Y(curr->position) + 1));
        ramification_list.push_back(tuple<int,int>(M_X(curr->position) + 1, M_Y(curr->position)));
        ramification_list.push_back(tuple<int,int>(M_X(curr->position) - 1, M_Y(curr->position)));
        ramification_list.push_back(tuple<int,int>(M_X(curr->position), M_Y(curr->position) - 1));

        while (!ramification_list.empty()) {
            // Draw a new posibility node
            tuple<int,int> current_possibility = ramification_list.front();
            ramification_list.erase(ramification_list.begin());
            // Check if the new possibility is valid or not
            PositionState state = map_->get_position_state(M_X(current_possibility), M_Y(current_possibility));
            if (state == GROUND || state == UNKNOWN) {
                begin = curr;
                // Check if that node was already on path from the solution or not, if yes, skip it.
                while (begin != nullptr) {
                    if (begin->position == current_possibility) break;
                    begin = begin->parent;
                }
                if (begin == nullptr) {
                    auto n = make_unique<Node>(current_possibility, curr, 0, curr->cost + 1);
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

vector<tuple<int, int>> MapAlgorithms::astar_shortest_path(const tuple<int,int>& start, const tuple<int, int>& end) {
    vector<tuple<int, int>> final_path;
    // start and end position must be a wall or an unknown place
    if (map_->get_position_state(M_X(start), M_Y(start)) == WALL ||
            map_->get_position_state(M_X(end), M_Y(end)) == WALL)
        return final_path;
    // Nodes might be in more than one path (shared)
    vector<unique_ptr<Node>> nodes_created_ownership;
    vector<Node*> open_nodes;
    auto start_node = make_unique<Node>(start, nullptr, heuristic_func_(start, end), 0);
    open_nodes.push_back(start_node.get());
    nodes_created_ownership.push_back(std::move(start_node));

    vector<tuple<int,int>> visited;
    Node *curr, *begin;
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
        ramification_list.push_back(tuple<int,int>(M_X(curr->position), M_Y(curr->position) + 1));
        ramification_list.push_back(tuple<int,int>(M_X(curr->position) + 1, M_Y(curr->position)));
        ramification_list.push_back(tuple<int,int>(M_X(curr->position) - 1, M_Y(curr->position)));
        ramification_list.push_back(tuple<int,int>(M_X(curr->position), M_Y(curr->position) - 1));

        while (!ramification_list.empty()) {
            // Draw a new posibility node
            auto current_possibility = ramification_list.front();
            ramification_list.erase(ramification_list.begin());
            // Check if the new possibility is valid or not
            PositionState state = map_->get_position_state(M_X(current_possibility), M_Y(current_possibility));
            if (state == GROUND) {
                begin = curr;
                // Check if that node was already on path from the solution or not, if yes, skip it.
                while (begin != nullptr) {
                    if (begin->position == current_possibility) break;
                    begin = begin->parent;
                }
                if (begin == nullptr) {
                    auto n = make_unique<Node>(current_possibility, curr,
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