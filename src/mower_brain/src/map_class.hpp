//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Date: November 2, 2017
// Filename: map_class.hpp
//
// Description: This declares a class where 
//
//////////////////////////////////////////////////////////////

#ifndef MAP_CLASS_HPP
#define MAP_CLASS_HPP

#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"



#define L0 0.2
#define L_OCC 1.0
#define L_FREE 0.0

#define MAP_HEIGHT 150		// 150 pixels tall (15m)
#define MAP_WIDTH 600		// 600 pixels wide (60m)
#define MAP_RESOLUTION 0.1	// each pixel represents 0.1m = 10cm

using std::vector;
typedef vector< vector<float> > Map_t;

class MyMap 
{
private:
  // make 2 instances of the map.
  Map_t map;
  
  ros::Publisher publisher;

public:
  MyMap();
  MyMap(ros::NodeHandle handle, int which_map);
  Map_t get_map() const { return map; }
  void set_map(Map_t new_map) { this->map = new_map; }

  /**
     looks in map for last column of unmowed stuff     
   */
  int find_unmowed();

  /**
     ADD 0'S to map_covered where map_covered not 1 and where map is less than 0.5
   */
  void add_uncut(MyMap main_map);

  /**
     marks map_covered with a 1 where ever lawnmower robot has been.
   */
  void mark_as_mowed(int x, int y);
		
  void publish(int which_map);
  float convert_to_probability(float log_odds);

  void draw_line(int x1, int y1, int x2, int y2);
  void draw_point(int x, int y, bool is_occupied);

  void cut_area(int x, int y, MyMap otherMap);

  void compare_results();
  
  float update_log_odds(float old_odds, bool is_occupied);
};

class Coordinate {
 public:
    int x;
    int y;

    Coordinate() : x(0), y(0) {}
    Coordinate(int nX, int nY) : x(nX), y(nY) {}
    Coordinate(const Coordinate& other) : x(other.x), y(other.y) {}

    ~Coordinate() {}

    bool operator==(const Coordinate& rval) const { return (x == rval.x && y == rval.y); }
    bool operator!=(const Coordinate& rval) const { return (x != rval.x || y != rval.y); }

    friend std::ostream& operator<<(std::ostream& os, const Coordinate& coord);

    Coordinate operator=(const Coordinate& rval) { x = rval.x; y = rval.y; return *this; }
};

// Make the Coordinate class hashable for STL objects
namespace std
{
    template<>
    struct hash<Coordinate> {
        size_t operator()(const Coordinate & obj) const {
            // hash using Cantor's Enumeration of Pairs
            return hash<int>()(((obj.x + obj.y)*(obj.x + obj.y + 1)/2) + obj.y);
        }
    };
}


#endif // MAP_CLASS_HPP

//////////////////////////////////////////////////////////////
