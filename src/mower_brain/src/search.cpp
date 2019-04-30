#include <unordered_set>
#include <queue>
#include <list>
#include <utility>
#include <iostream>
#include <vector>
#include "search.hpp"

class State {
 public:
    Coordinate coord;
    std::list<Coordinate> coord_list;     
    int cost;
    
    State() {
        coord = Coordinate();
        coord_list = std::list<Coordinate>();
        cost = 0;
    }
    State(const Coordinate& nCoord, const std::list<Coordinate>& nCoord_list) { 
        coord = Coordinate(nCoord);
        coord_list = std::list<Coordinate>(nCoord_list);
        cost = 0;
    }
    State(const Coordinate& nCoord, const std::list<Coordinate>& nCoord_list, const int nCost) { 
        coord = Coordinate(nCoord);
        coord_list = std::list<Coordinate>(nCoord_list);
        cost = nCost;
    }

    State(const State& nState) {
        coord = Coordinate(nState.coord);
        coord_list = std::list<Coordinate>(nState.coord_list);
        cost = nState.cost;
    }

    ~State() {}

    State operator=(const State& rval) { 
        coord = Coordinate(rval.coord);
        coord_list = std::list<Coordinate>(rval.coord_list);
        cost = rval.cost;
        return *this;
    }

    bool operator<(const State& rval) const { return cost < rval.cost; }
    bool operator>(const State& rval) const { return cost > rval.cost; }
    bool operator>=(const State& rval) const { return cost >= rval.cost; }
    bool operator<=(const State& rval) const { return cost <= rval.cost; }
    bool operator==(const State& rval) const { return cost == rval.cost; }
    bool operator!=(const State& rval) const { return cost != rval.cost; }
};


// typedef std::pair< Coordinate, std::list<Coordinate>* > State;

// This does not work currently, and I think that it is because the std::priority queue is an unstable algorithm that makes 
// no guarantees about how elements with the same priority are stored & retrieved from the queue. I'm working on a custom one
// that does this now.
std::list<Coordinate> breadth_first_search(Coordinate start, Coordinate goal, const MyMap& mower_map, const MyMap& main_map) {
    // const int DEPTH_LIMIT = 30;
    auto wall_map = main_map.get_map();
    std::queue<State> frontier;
    std::unordered_set<Coordinate> explored_set;
    std::list<Coordinate> result;
    State nextState;
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
        if (!explored_set.count(nextState.coord)) { //explored_set.find(nextState.coord) == explored_set.end()
            explored_set.insert(nextState.coord);

            for (int i = -1; i <= 1; i += 2) {
                if (i + nextState.coord.x >= 0 && i + nextState.coord.x < wall_map.size() && wall_map[i + nextState.coord.x][nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    nextState.coord_list.push_back(nextState.coord);
                    frontier.push(State(Coordinate(i + nextState.coord.x, nextState.coord.y), nextState.coord_list));
                    nextState.coord_list.pop_back();
                }
            }
            for (int j = -1; j <= 1; j += 2) {
                if (j + nextState.coord.y >= 0 && j + nextState.coord.y < wall_map[nextState.coord.x].size() && wall_map[nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    nextState.coord_list.push_back(nextState.coord);
                    frontier.push(State(Coordinate(nextState.coord.x, j + nextState.coord.y), nextState.coord_list));
                    nextState.coord_list.pop_back();
                }
            }
        }
    }
    std::cout << "Could Not Find Goal..." << std::endl;
    return nextState.coord_list;
}


// frontier, explored_set = util.PriorityQueue(), set()
//     frontier.push((problem.getStartState(), list()), 0)
//     while not frontier.isEmpty():
//         node, actions = frontier.pop()
//         if problem.isGoalState(node):
//             return actions
//         if node not in explored_set:
//             explored_set.add(node)
//             for next_node, action, cost in problem.getSuccessors(node):
//                 frontier.push((next_node, actions + [action]), cost + problem.getCostOfActions(actions))
//     return None

std::list<Coordinate> uniform_cost_search(Coordinate start, Coordinate goal, MyMap mower_map, MyMap main_map) {
    // const int DEPTH_LIMIT = 30;
    auto wall_map = main_map.get_map();
    std::priority_queue<State> frontier;
    std::unordered_set<Coordinate> explored_set;
    std::list<Coordinate> result;
    State nextState;
    nextState.coord = start;
    frontier.push(nextState);
    while (!frontier.empty()) {
        // std::cout << "at top of loop" << std::endl;
        nextState = frontier.top();
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
                    nextState.coord_list.push_back(nextState.coord);
                    frontier.push(State(Coordinate(i + nextState.coord.x, nextState.coord.y), nextState.coord_list));
                    nextState.coord_list.pop_back();
                }
            }
            for (int j = -1; j <= 1; j += 2) {
                if (j + nextState.coord.y >= 0 && j + nextState.coord.y < wall_map[nextState.coord.x].size() && wall_map[nextState.coord.x][j + nextState.coord.y] <= 0.5) {
                    // not a wall, add it to the possibilities
                    nextState.coord_list.push_back(nextState.coord);
                    frontier.push(State(Coordinate(nextState.coord.x, j + nextState.coord.y), nextState.coord_list));
                    nextState.coord_list.pop_back();
                }
            }
        }
    }
    std::cout << "Could Not Find Goal..." << std::endl;
    return nextState.coord_list;
}
