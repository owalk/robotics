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

    // remove later and have set by function
    robot.coord_goal.x = 120;
    robot.coord_goal.y = 70;
    std::cout << "Beginning BFS" << std::endl;
    auto thing = breadth_first_search(Coordinate(robot.get_x_position_raw(), robot.get_y_position_raw()),
                         Coordinate(robot.coord_goal.x / 10.0, robot.coord_goal.y / 10.0),
                         mower_map,
                         main_map);

    for (auto item : thing) {
        std::cout << "( " << item.x << ", " << item.y << ") ; ";
    }
    ///////////////
    std::cout << "Finished BFS" << std::endl;
    
    while(ros::ok()) 
    {	         
      robot_x = robot.get_x_position_relative();
      robot_y = robot.get_y_position_relative();
     
      //stop timer 2
      gettimeofday(&t2, NULL);
      
      elapsedTime = (t2.tv_sec - t1.tv_sec);

      int i;
      int t = 15;
      /* old code
      int minutes = 2;
      for(i=0; i<t; i++){

        if(elapsedTime >= 60*i*minutes && elapsedTime < (minutes*60*i)+240) // 6 - 9 minutes
          robot.floor_cover(1.7+(i*0.5));

        if(elapsedTime == (i*minutes*60)){
          std::cout << "Time passed:  "<< i*minutes <<"minute\n";
          mower_map.compare_results();
        }
      }
      mower_map.compare_results();
      */

      // change state by one to the left
      // -1 to the x position.

      // if goal reached, updated goal
      if( robot_x != robot.coord_goal.x &&
          robot_y != robot.coord_goal.y)
        {
          // go to goal
          robot.seek_goal(); 
        }

      
      
      // write out 1's to second map where cut
      mower_map.cut_area(robot_x, robot_y, main_map);

      // write out 0's where uncut but known as part of the map
      mower_map.add_uncut(main_map);

      // end of timer message
      if( elapsedTime  > t*60*4){ // t * 4 minutes
        std::cout << "Time passed:  "<< i*4 <<"minute\n";
        std::cout << "Ending Program!\n";
        return 0; // end the node
      }
      
      
      
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

