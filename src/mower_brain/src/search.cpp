#include <unordered_set>
#include <queue>
#include <list>
#include <utility>
#include <iostream>
#include <vector>
#include "search.hpp"
#include "stable_priority_queue.hpp"


int Search_Problem::get_grass_cost(double grass_value) {
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


std::list<Coordinate> Search_Problem::breadth_first_search(const Coordinate& start) {
    Map_t wall_map = mpWall_map->get_map();
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
        if (nextState == mGoalState) {
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
                    newState = State(Coordinate(i + nextState.coord.x, nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    frontier.push(newState);
                }
            }
            for (int j = -1; j <= 1; j += 2) {
                if (j + nextState.coord.y >= 0 && j + nextState.coord.y < wall_map[nextState.coord.x].size() && 
                    wall_map[nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    newState = State(Coordinate(nextState.coord.x, j + nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    frontier.push(newState);
                }
            }
        }
    }
    std::cout << "Could Not Find Goal..." << std::endl;
    return nextState.coord_list;
}


std::list<Coordinate> Search_Problem::uniform_cost_search(const Coordinate& start) {
    Map_t wall_map = mpWall_map->get_map();
    Map_t grass_map = mpMower_map->get_map();
    Stable_Priority_Queue<State> frontier;
    std::unordered_set<Coordinate> explored_set;
    std::list<Coordinate> result;
    State nextState;
    State newState;
    int cost;
    nextState.coord = start;
    frontier.push(0, nextState);
    while (!frontier.empty()) {
        nextState = frontier.top();
        frontier.pop();
        if (nextState == mGoalState) {
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
                    newState = State(Coordinate(i + nextState.coord.x, nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    cost = get_grass_cost(grass_map[newState.coord.x][newState.coord.y]);
                    frontier.push(cost, newState);
                }
            }
            for (int j = -1; j <= 1; j += 2) {
                if (j + nextState.coord.y >= 0 && j + nextState.coord.y < wall_map[nextState.coord.x].size() &&
                    wall_map[nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    newState = State(Coordinate(nextState.coord.x, j + nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    cost = get_grass_cost(grass_map[newState.coord.x][newState.coord.y]);
                    frontier.push(cost, newState);
                }
            }
        }
    }
    std::cout << "Could Not Find Goal..." << std::endl;
    return nextState.coord_list;
}


std::list<Coordinate> Search_Problem::a_star_search(const Coordinate& start, Heuristic_t heuristic) {
    Map_t wall_map = mpWall_map->get_map();
    Map_t grass_map = mpMower_map->get_map();
    Stable_Priority_Queue<State> frontier;
    std::unordered_set<Coordinate> explored_set;
    std::list<Coordinate> result;
    State nextState;
    State newState;
    int cost;
    nextState.coord = start;
    frontier.push(0, nextState);
    while (!frontier.empty()) {
        nextState = frontier.top();
        frontier.pop();
        if (nextState == mGoalState) {
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
                    newState = State(Coordinate(i + nextState.coord.x, nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    cost = get_grass_cost(grass_map[newState.coord.x][newState.coord.y]) 
                           + heuristic(newState, mGoalState, grass_map, wall_map);
                    frontier.push(cost, newState);
                }
            }
            for (int j = -1; j <= 1; j += 2) {
                if (j + nextState.coord.y >= 0 && j + nextState.coord.y < wall_map[nextState.coord.x].size() &&
                    wall_map[nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    newState = State(Coordinate(nextState.coord.x, j + nextState.coord.y), nextState.coord_list);
                    newState.coord_list.push_back(nextState.coord);
                    cost = get_grass_cost(grass_map[newState.coord.x][newState.coord.y])
                           + heuristic(newState, mGoalState, grass_map, wall_map);
                    frontier.push(cost, newState);
                }
            }
        }
    }
    std::cout << "Could Not Find Goal..." << std::endl;
    return nextState.coord_list;
}
