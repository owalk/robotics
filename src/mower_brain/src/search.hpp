#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <list>
#include "map_class.hpp"

std::list<Coordinate> breadth_first_search(Coordinate start, Coordinate goal, MyMap mower_map, MyMap main_map);

#endif  // SEARCH_HPP
