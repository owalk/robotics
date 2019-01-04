//////////////////////////////////////////////////////////////
//
// Programmer: Oliver Walker
// Date: January 3, 2019
// Filename: robot_class.cpp
//
// Description: This implements a class where 
//
//////////////////////////////////////////////////////////////

#include "robot_class.hpp"

// CONSTRUCTOR (default)
// initializes all the laser ranges to -1, an error state
Robot::Robot() 
{
	this->x_position = -1.0;
	this->y_position = -1.0;
	this->quaternion = quaternion;

	this->west = -1.0;
	this->northwest = -1.0;
	this->north = -1.0;
	this->northeast = -1.0;
	this->east = -1.0;
}

// CONSTRUCTOR 
// handle: node handle for pub/sub setup
// sets up the subscriber and publisher with the given node handle
Robot::Robot(ros::NodeHandle handle)
{
	this->x_position = -1.0;
	this->y_position = -1.0;
	this->quaternion = quaternion;

	this->west = -1.0;
	this->northwest = -1.0;
	this->north = -1.0;
	this->northeast = -1.0;
	this->east = -1.0;

	this-> publisher = handle.advertise<geometry_msgs::Twist>("robot/cmd_vel", 10);
	this-> laser_subscriber = handle.subscribe<sensor_msgs::LaserScan>("robot/base_scan", 10, &Robot::laser_callback, this);
	this-> position_subscriber = handle.subscribe<nav_msgs::Odometry>("stage/base_pose_ground_truth", 10, &Robot::position_callback, this);
}
	
// EXPLORE FUNCTION: modified by Oliver.
// wall follows when not up against top


void Robot::explore() 
{

  // if to close, swivel to the left
  if (this->northeast < 2 || this->north < 2) // if past right bound, go left
    this->avoid_obstacle(); 
  else
    this->right_wall_follow();

}
//written by oliver
void Robot::floor_cover(float dist){
  
  if(this->northeast < 0.5 ||
     this->north < 0.5 ||
     this->northwest < 0.5) // prioritize not getting stuck on a wall
    this->avoid_obstacle();
  else{ //else stay in the required dist from wall
    if(this->northeast < dist || this-> north < dist)
      this->drift_right();
    
    else if (this->northeast > dist+0.5 || this->north > dist+0.5) // if past right bound, go left
      this->drift_left();
  }
}

// Written by oliver, function todrive over a spot on the map
void Robot::tilt(){

  geometry_msgs::Twist msg;  
  msg.angular.z = 1.0; // change angle
  this->publisher.publish(msg);
}

// Written by oliver, function todrive over a spot on the map
void Robot::plus_x(){

  geometry_msgs::Twist msg;  
  msg.angular.z = 0;
  msg.linear.x = 3;
  this->publisher.publish(msg);
}


// Written by oliver, function todrive over a spot on the map
void Robot::minus_x(){

  geometry_msgs::Twist msg;  
  msg.angular.z = 0;
  msg.linear.x = -3;
  this->publisher.publish(msg);
}



// RIGHT WALL FOLLOW FUNCTION
// publishes a twist message enabling the robot to right wall follow
void Robot::right_wall_follow() 
{
	geometry_msgs::Twist msg;
	msg.linear.x = 3.0; // was 3.0
	msg.angular.z = 1.0 - this->east;
	this->publisher.publish(msg);
}

// AVOID OBSTACLE FUNCTION
// publishes a twist message enabling the robot to avoid obstacles if about to crash
void Robot::avoid_obstacle() 
{
	geometry_msgs::Twist msg_turnaway;
	msg_turnaway.linear.x = 0.0;
	msg_turnaway.angular.z = 3.0;
	this->publisher.publish(msg_turnaway);
}



void Robot::drift_left() 
{
	geometry_msgs::Twist msg_turnaway;
	msg_turnaway.linear.x = 1.0;
	msg_turnaway.angular.z = -3.0;
	this->publisher.publish(msg_turnaway);
}


void Robot::drift_right() 
{
	geometry_msgs::Twist msg_turnaway;
	msg_turnaway.linear.x = 1.0;
	msg_turnaway.angular.z = 3.0;
	this->publisher.publish(msg_turnaway);
}

// LASER SENSOR CALLBACK (sets the laser beam lengths)
// msg: the message read by the subscriber
// returns: n/a
void Robot::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	this->west = msg->ranges[4];
	this->northwest = msg->ranges[3];
	this->north = msg->ranges[2];
	this->northeast = msg->ranges[1];
	this->east = msg->ranges[0];
}

// POSITION SENSOR CALLBACK (sets the position and orientation)
// msg: the message read by the subscriber
// returns: n/a
void Robot::position_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	this->x_position = msg->pose.pose.position.x;
	this->y_position = msg->pose.pose.position.y;

	tf::Quaternion new_q(
		msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);

	this->quaternion = new_q;
}

//////////////////////////////////////////////////////////////
