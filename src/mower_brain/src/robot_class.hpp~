//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Date: November 2, 2017
// Filename: robot_class.hpp
//
// Description: This declares a class where 
//
//////////////////////////////////////////////////////////////

#ifndef ROBOT_CLASS_HPP
#define ROBOT_CLASS_HPP

#include <sys/time.h> // for timings
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

#define SCALE 10.0

class Robot 
{
private: 
  float x_position;
  float y_position;
  tf::Quaternion quaternion;
		
  float west;
  float northwest;
  float north;
  float northeast;
  float east;
		
  ros::Publisher publisher;
  ros::Subscriber laser_subscriber;
  ros::Subscriber position_subscriber;

public:
  Robot();
  Robot(ros::NodeHandle handle);
	
  float get_x_position_relative() { return (30.0 + this->x_position) * SCALE; }
  float get_y_position_relative() { return (7.5 + this->y_position) * SCALE; }
  float get_angle() { return this->quaternion.getAngle(); }
		
  float get_west() { return this->west; }
  float get_northwest() { return this->northwest; }
  float get_north() { return this->north; }
  float get_northeast() { return this->northeast; }
  float get_east() { return this->east; }
		
  void explore();

  // written by oliver
  void floor_cover(float dist);
  void up_down();
  void tilt();

  void plus_x();
  void minus_x();
  
  void right_wall_follow();
  void avoid_obstacle();
  void drift_right();
  void drift_left();
  
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void position_callback(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif // ROBOT_CLASS_HPP

//////////////////////////////////////////////////////////////
