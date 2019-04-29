#include <unordered_set>
#include <queue>
#include <list>
#include <utility>
#include <iostream>
#include <vector>
#include "search.hpp"

class State {
 public:
    State() {}
    State(Coordinate nCoord, std::list<Coordinate> nCoord_list) { 
        coord = nCoord;
        coord_list = nCoord_list;
    }
    ~State() {}
    Coordinate coord;
    std::list<Coordinate> coord_list;     
};


// typedef std::pair< Coordinate, std::list<Coordinate>* > State;

std::list<Coordinate> breadth_first_search(Coordinate start, Coordinate goal, MyMap mower_map, MyMap main_map) {
    // const int DEPTH_LIMIT = 30;
    auto wall_map = main_map.get_map();
    std::queue<State> frontier;
    std::unordered_set<Coordinate> explored_set;
    std::list<Coordinate> result;
    State nextState;
    nextState.coord = start;
    frontier.push(nextState);
    std::cout << "beginning loop" << std::endl;
    while (!frontier.empty()) {
        // std::cout << "at top of loop" << std::endl;
        nextState = frontier.front();
        frontier.pop();
        if (nextState.coord == goal) {
            std::cout << "Found goal state!" << std::endl;
            nextState.coord_list.push_back(nextState.coord);
            return nextState.coord_list;
        }
        if (explored_set.find(nextState.coord) == explored_set.end()) {
            explored_set.insert(nextState.coord);
            // push things into the queue
            // std::cout << "putting things in the queue" << std::endl;
            for (int i = -1; i <= 1; i += 2) {
                for (int j = -1; j <= 1; j += 2) {
                    // std::cout << "checking (" << i + nextState.coord.x << ", " << j + nextState.coord.x << ")" << std::endl;
                    if (i + nextState.coord.x >= 0 && j + nextState.coord.y >= 0 && i + nextState.coord.x < wall_map.size() && j + nextState.coord.y < wall_map[i].size()  
                        && wall_map[i + nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                        // not a wall, add it to the possibilities
                        nextState.coord_list.push_back(nextState.coord);
                        frontier.push(State(Coordinate(i + nextState.coord.x, j + nextState.coord.y), nextState.coord_list));
                    }
                }
            }
        }
    }
    std::cout << "Returning...." << std::endl;
    return nextState.coord_list;
    /*
    frontier, explored_set = util.Queue(), set()
        frontier.push((problem.getStartState(), list()))
        while not frontier.isEmpty():
            node, actions = frontier.pop()
            if problem.isGoalState(node):
                return actions
            if node not in explored_set:
                explored_set.add(node)
                for next_node, action, _ in problem.getSuccessors(node):
                    frontier.push((next_node, actions + [action]))
        return None
    */

}

