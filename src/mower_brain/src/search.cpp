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
    while (!frontier.empty()) {
        // std::cout << "at top of loop" << std::endl;
        nextState = frontier.front();
        frontier.pop();
        if (nextState.coord == goal) {
            std::cout << "Found goal state!" << std::endl;
            nextState.coord_list.push_back(nextState.coord);
            return nextState.coord_list;
        }
        if (!explored_set.count(nextState.coord)) { //explored_set.find(nextState.coord) == explored_set.end()
            explored_set.insert(nextState.coord);

            for (int i = -1; i <= 1; i += 2) {
                if (i + nextState.coord.x >= 0 && i + nextState.coord.x < wall_map.size() && wall_map[i + nextState.coord.x][nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    // if (nextState.coord_list.back() == nextState.coord) {
                    //     std::cout << nextState.coord_list.back() << " Is equal to " << nextState.coord << std::endl;
                    // }
                    nextState.coord_list.push_back(nextState.coord);
                    frontier.push(State(Coordinate(i + nextState.coord.x, nextState.coord.y), nextState.coord_list));
                    nextState.coord_list.pop_back();
                }
            }
            for (int j = -1; j <= 1; j += 2) {
                if (j + nextState.coord.y >= 0 && j + nextState.coord.y < wall_map[nextState.coord.x].size() && wall_map[nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    
                    // if (nextState.coord_list.back() == nextState.coord) {
                    //     std::cout << nextState.coord_list.back() << " Is equal to " << nextState.coord << std::endl;
                    // }

                    nextState.coord_list.push_back(nextState.coord);
                    frontier.push(State(Coordinate(nextState.coord.x, j + nextState.coord.y), nextState.coord_list));
                    nextState.coord_list.pop_back();
                }
            }
        }
    }
    std::cout << "Could Not Find Goal..." << std::endl;
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

