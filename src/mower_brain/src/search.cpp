#include <unordered_set>
#include <queue>
#include <list>
#include <utility>
#include <iostream>
#include <vector>
#include "search.hpp"
#include "stable_priority_queue.hpp"

const int UNCUT_GRASS_COST = 1;
const int UNKNOWN_GRASS_COST = 1; // same as uncut grass, assumes all unknown to be uncut
const int CUT_GRASS_COST = 5;

class State {
 public:
    Coordinate coord;
    std::list<Coordinate> coord_list;     
    int cost;
    
    State() {
        coord = Coordinate();
        coord_list = std::list<Coordinate>();
    }

    State(const Coordinate& nCoord, const std::list<Coordinate>& nCoord_list) { 
        coord = Coordinate(nCoord);
        coord_list = std::list<Coordinate>(nCoord_list);
    }

    State(const State& nState) {
        coord = Coordinate(nState.coord);
        coord_list = std::list<Coordinate>(nState.coord_list);
    }

    ~State() {}

    State operator=(const State& rval) { 
        coord = Coordinate(rval.coord);
        coord_list = std::list<Coordinate>(rval.coord_list);
        return *this;
    }
};

int get_grass_cost(double grass_value) {
    if (grass_value <= 0.1) {
        // cut grass
        return CUT_GRASS_COST;
    } else if (grass_value >= 0.9) {
        // uncut grass
        return UNCUT_GRASS_COST;
    } else {
        // unknown space
        return UNKNOWN_GRASS_COST;
    }
}


std::list<Coordinate> breadth_first_search(Coordinate start, Coordinate goal, const MyMap& mower_map, const MyMap& main_map) {
    // const int DEPTH_LIMIT = 30;
    auto wall_map = main_map.get_map();
    std::queue<State> frontier;
    std::unordered_set<Coordinate> explored_set;
    std::list<Coordinate> result;
    State nextState;
    State newState;
    nextState.coord = start;
    frontier.push(nextState);
    while (!frontier.empty()) {
        nextState = frontier.front();
        frontier.pop();
        if (nextState.coord == goal) {
            std::cout << "Found goal state!" << std::endl;
            nextState.coord_list.push_back(nextState.coord);
            return nextState.coord_list;
        }
        if (!explored_set.count(nextState.coord)) {
            explored_set.insert(nextState.coord);

            for (int i = -1; i <= 1; i += 2) {
                if (i + nextState.coord.x >= 0 && i + nextState.coord.x < wall_map.size() &&
                    wall_map[i + nextState.coord.x][nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    // nextState.coord_list.push_back(nexttate.coord);
                    newState = State(Coordinate(i + nextState.coord.x, nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    frontier.push(newState);
                    // nextState.coord_list.pop_back();
                }
            }
            for (int j = -1; j <= 1; j += 2) {
                if (j + nextState.coord.y >= 0 && j + nextState.coord.y < wall_map[nextState.coord.x].size() && 
                    wall_map[nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    // nextState.coord_list.push_back(nextState.coord);
                    newState = State(Coordinate(nextState.coord.x, j + nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    frontier.push(newState);
                    // nextState.coord_list.pop_back();
                }
            }
        }
    }
    std::cout << "Could Not Find Goal..." << std::endl;
    return nextState.coord_list;
}


std::list<Coordinate> uniform_cost_search(Coordinate start, Coordinate goal, MyMap mower_map, MyMap main_map) {
    // const int DEPTH_LIMIT = 30;
    auto wall_map = main_map.get_map();
    auto grass_map = mower_map.get_map();
    Stable_Priority_Queue<State> frontier;
    std::unordered_set<Coordinate> explored_set;
    std::list<Coordinate> result;
    State nextState;
    State newState;
    int cost;
    nextState.coord = start;
    frontier.push(0, nextState);
    while (!frontier.empty()) {
        // std::cout << "at top of loop" << std::endl;
        nextState = frontier.top();
        frontier.pop();
        if (nextState.coord == goal) {
            std::cout << "Found goal state!" << std::endl;
            nextState.coord_list.push_back(nextState.coord);
            return nextState.coord_list;
        }
        if (!explored_set.count(nextState.coord)) {
            explored_set.insert(nextState.coord);

            for (int i = -1; i <= 1; i += 2) {
                if (i + nextState.coord.x >= 0 && i + nextState.coord.x < wall_map.size() && 
                    wall_map[i + nextState.coord.x][nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    // nextState.coord_list.push_back(nextState.coord);
                    newState = State(Coordinate(i + nextState.coord.x, nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    cost = get_grass_cost(grass_map[newState.coord.x][newState.coord.y]);
                    frontier.push(cost, newState);
                    // nextState.coord_list.pop_back();
                }
            }
            for (int j = -1; j <= 1; j += 2) {
                if (j + nextState.coord.y >= 0 && j + nextState.coord.y < wall_map[nextState.coord.x].size() &&
                    wall_map[nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    // nextState.coord_list.push_back(nextState.coord);
                    newState = State(Coordinate(nextState.coord.x, j + nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    cost = get_grass_cost(grass_map[newState.coord.x][newState.coord.y]);
                    frontier.push(cost, newState);
                    // nextState.coord_list.pop_back();
                }
            }
        }
    }
    std::cout << "Could Not Find Goal..." << std::endl;
    return nextState.coord_list;
}
