////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Oliver Walker
// Date: January 3, 2019
// Filename: mapperer.cpp
//
// Description: This file contains the main loop where our robot create a
// map of the world given and then executes its exploratory logic.
//
////////////////////////////////////////////////////////////////////////////////
#define _USE_MATH_DEFINES

#include <cmath>
#include <math.h>
#include <visualization_msgs/Marker.h> // for drawing path
#include "ros/ros.h"
#include "robot_class.hpp"
#include "map_class.hpp"
#include "search.hpp"
    
int main(int argc, char **argv)
{
      // initialize node and create the node handle
      ros::init(argc, argv, "controller");
      ros::NodeHandle nh;
    
    // make a new robot and a new map
    Robot robot(nh);
    MyMap main_map(nh,1);
    MyMap mower_map(nh,2); // holds .1 for cut, .9 for uncut, .5 for unknown.
    ros::Rate loop_rate(10);  	
    ros::spinOnce();

    // declare some variables for later
    int robot_x, robot_y;
    int x, y;

    // code from stack overflow on timers
    struct timeval t1, t2;
    double elapsedTime;

    // start timer
    gettimeofday(&t1, NULL);

    // initial coordinates to start the search
    robot.coord_goal.x = 190;
    robot.coord_goal.y = 50;

    std::cout << "Beginning BFS" << std::endl;
    auto coord_goal_list = breadth_first_search(Coordinate(robot.get_x_position_raw(), robot.get_y_position_raw()),
				      Coordinate(robot.coord_goal.x / 10.0, robot.coord_goal.y / 10.0),
				      mower_map,
				      main_map);

    for (auto item : coord_goal_list) {
      std::cout << "( " << item.x << ", " << item.y << ") ; ";
    }
    ///////////////
    std::cout << "Finished BFS" << std::endl;

    robot.coord_goal.x = coord_goal_list.front().x;
    robot.coord_goal.y = coord_goal_list.front().y;

    
    while(ros::ok()) 
    {
      int x_raw = (int)robot.get_x_position_relative()/10;
      int y_raw = (int)robot.get_y_position_relative()/10;
     
      //stop timer 2
      gettimeofday(&t2, NULL);
      
      elapsedTime = (t2.tv_sec - t1.tv_sec);    
          

      if( x_raw != robot.coord_goal.x &&
	  y_raw != robot.coord_goal.y){
	// go to goal
	std::cout << "search robot seeking goal (" << robot.coord_goal.x <<","<< robot.coord_goal.y<< ")\n";
	std::cout << "search robot currently at (" << x_raw <<","<< y_raw<< ")\n\n";
	robot.seek_goal(); 
      }
      else{ // set goal
	
	// follow each point from search function to the goal
	std::cout << "robot reached (" <<x_raw<<
	  ","<<y_raw<<")\n";
	robot.stop();
	coord_goal_list.pop_front();
	robot.coord_goal.x = coord_goal_list.front().x;
	robot.coord_goal.y = coord_goal_list.front().y;
	std::cout << "robot now seeking (" <<robot.coord_goal.x<<
	  ","<<robot.coord_goal.y<<")\n\n";
	      
	
      }
      /////////////////////////////////////////////
	
      if(coord_goal_list.front().x == 0 &&
	 coord_goal_list.front().y == 0){
	// end of coordinate list has been hit. end program
	return 0;
	}

           
      
      // write out 1's to second map where cut
      mower_map.cut_area(robot_x, robot_y, main_map);

      // write out 0's where uncut but known as part of the map
      mower_map.add_uncut(main_map);


      //down here we can do a end of program condition to return 0 from
      // if something
      // return 0;

      robot_x = robot.get_x_position_relative();
      robot_y = robot.get_y_position_relative();
      
      main_map.draw_point(robot_x, robot_y, false);
            
      // get the west sensor beam
      x = robot.get_west() * 10.0 * cos(robot.get_angle() + M_PI / 2) + robot_x;
      y = robot.get_west() * 10.0 * sin(robot.get_angle() + M_PI / 2) + robot_y;
      main_map.draw_line(robot_x, robot_y, x, y);
      if (robot.get_west() != 5.0) 
        {
          main_map.draw_point(x - 1, y - 1, true);
          main_map.draw_point(x - 1, y, true);
          main_map.draw_point(x, y - 1, true);
          main_map.draw_point(x, y, true);
        }
        
      // get the northwest sensor beam	
      x = robot.get_northwest() * 10.0 * cos(robot.get_angle() + M_PI / 4) + robot_x;
      y = robot.get_northwest() * 10.0 * sin(robot.get_angle() + M_PI / 4) + robot_y;
      main_map.draw_line(robot_x, robot_y, x, y);
      if (robot.get_northwest() != 5.0)
        {
          main_map.draw_point(x - 1, y - 1, true);
          main_map.draw_point(x - 1, y, true);
          main_map.draw_point(x, y - 1, true);
          main_map.draw_point(x, y, true);
        }
    
      // get the north sensor beam	
      x = robot.get_north() * 10.0 * cos(robot.get_angle()) + robot_x;
      y = robot.get_north() * 10.0 * sin(robot.get_angle()) + robot_y;
      main_map.draw_line(robot_x, robot_y, x, y);
      if (robot.get_north() != 5.0) 
        {
          main_map.draw_point(x - 1, y - 1, true);
          main_map.draw_point(x - 1, y, true);
          main_map.draw_point(x, y - 1, true);
          main_map.draw_point(x, y, true);
        }
    
      // get the northeast sensor beam	
      x = robot.get_northeast() * 10.0 * cos(robot.get_angle() - M_PI / 4) + robot_x;
      y = robot.get_northeast() * 10.0 * sin(robot.get_angle() - M_PI / 4) + robot_y;
      main_map.draw_line(robot_x, robot_y, x, y);
      if (robot.get_northeast() != 5.0)  
        {
          main_map.draw_point(x - 1, y - 1, true);
          main_map.draw_point(x - 1, y, true);
          main_map.draw_point(x, y - 1, true);
          main_map.draw_point(x, y, true);
        }
    
      // get the east sensor beam	
      x = robot.get_east() *  10.0 * cos(robot.get_angle() - M_PI / 2) + robot_x;
      y = robot.get_east() * 10.0 * sin(robot.get_angle() - M_PI / 2) + robot_y;
      main_map.draw_line(robot_x, robot_y, x, y);
      if (robot.get_east() != 5.0) 
        {
          main_map.draw_point(x - 1, y - 1, true);
          main_map.draw_point(x - 1, y, true);
          main_map.draw_point(x, y - 1, true);
          main_map.draw_point(x, y, true);
        }
    
      // publish the map
      main_map.publish(1);
      mower_map.publish(2);

      // spin and sleep
      ros::spinOnce();
      loop_rate.sleep();
    }
        
      return 0;
}

////////////////////////////////////////////////////////////////////////////////

