#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <list>
#include "map_class.hpp"
#include <algorithm>

class State {
     public:
        Coordinate coord;
        std::list<Coordinate> coord_list;     
        
        State() {
            coord = Coordinate();
            coord_list = std::list<Coordinate>();
        }

        State(const Coordinate& nCoord) : coord(nCoord) {}

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

        bool operator==(const State& rval) { return coord == rval.coord; }
        bool operator!=(const State& rval) { return coord != rval.coord; }
    };

// typedef int Heuristic_t;
typedef int (*Heuristic_t)(const State&, const State&, const Map_t&, const Map_t&);

class Search_Problem {
 private:
    // variables
    State mGoalState;
    MyMap* mpMower_map;
    MyMap* mpWall_map;

    // constants
    static const int UNCUT_GRASS_COST = 1;
    static const int UNKNOWN_GRASS_COST = 1; // same as uncut grass, assumes all unknown to be uncut
    static const int CUT_GRASS_COST = 5;

    // functions
    static int get_grass_cost(double grass_value);

 public:
    // Constructors (no default)
    Search_Problem(const State& goal, MyMap* const pMower_map, MyMap* const pWall_map) : 
        mGoalState(goal), mpMower_map(pMower_map), mpWall_map(pWall_map) {}

    // Class methods
    State get_goal() const { return mGoalState; }
    void set_goal(const State& goal) { mGoalState = goal; }

    void set_mower_map(MyMap* const mower_map) { mpMower_map = mower_map; }
    void set_wall_map(MyMap* const wall_map) { mpWall_map = wall_map; }
    
    // Heuristics
    static int null_heuristic(const State& state, const State& goal, const Map_t& mower_map, const Map_t& wall_map) {
        return 0;
    }

    // Search Algorithms
    std::list<Coordinate> breadth_first_search(const Coordinate& start);
    std::list<Coordinate> uniform_cost_search(const Coordinate& start);
    std::list<Coordinate> a_star_search(const Coordinate& start, Heuristic_t heuristic=null_heuristic);
};

#endif  // SEARCH_HPP
