//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Date: November 2, 2017
// Filename: map_class.cpp
//
// Description: This declares a class where 
//
//////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"
#include "map_class.hpp"
#include <iostream> //cout

// DEFAULT MAP CONSTRUCTOR
// create a map initialized to all unknown values
// very slightly modified by oliver
MyMap::MyMap() 
{
	for (int i = 0; i < MAP_HEIGHT; i++) 
	{
		vector<float> new_vector;
		
		for (int j = 0; j < MAP_WIDTH; j++) 
		{
			new_vector.push_back(-1);
		}
		this->map.push_back(new_vector);
	}
}

MyMap::MyMap(ros::NodeHandle handle, int which_map) 
{
  for (int i = 0; i < MAP_HEIGHT; i++) 
    {      
      vector<float> new_vector;
      //      vector<int> new_vector2; 
      for (int j = 0; j < MAP_WIDTH; j++) 
	{
	  
	  new_vector.push_back(0.5);
	  //  new_vector2.push_back(-1); // set to all unknown
	}
      
      this->map.push_back(new_vector);
      //this->map_covered.push_back(new_vector2);
    }
  
  // make another pub for uncut map
  // move map logic to main to have a second publisher to show it
  if(which_map == 1)
      this->publisher = handle.advertise<nav_msgs::OccupancyGrid>("/map", 10);
  else
      this->publisher = handle.advertise<nav_msgs::OccupancyGrid>("/map2", 10);
}

// PUBLISH FUNCTION
// publish the map member as an occupancy grid
void MyMap::publish(int which_map) 
{
	// create a header, populate the fields.
	std_msgs::Header header = std_msgs::Header();
	header.stamp = ros::Time::now();
	if(which_map == 1)
	  header.frame_id = "map2";
	else
	  header.frame_id = "map";
	
	// create the map meta data
	nav_msgs::MapMetaData metaD = nav_msgs::MapMetaData();
	metaD.map_load_time = ros::Time::now();
	metaD.resolution = MAP_RESOLUTION;
	metaD.width = MAP_WIDTH;
	metaD.height = MAP_HEIGHT;

	// create the map from the header and the meta data
	nav_msgs::OccupancyGrid oGrid = nav_msgs::OccupancyGrid();
	oGrid.header = header;
	oGrid.info = metaD;

	// add the probabilities to the map
	for (int i = 0; i < MAP_HEIGHT; i++)
	{
		for (int j = 0; j < MAP_WIDTH; j++) 
		{
			if(which_map == 1){	
				int probability = this->map[i][j];	
				oGrid.data.push_back(probability);
				}
					
			else{ 
				int probability = convert_to_probability(this->map[i][j]);
				oGrid.data.push_back(probability);}
		}
	}

	// publush the occupancy grid
	this->publisher.publish(oGrid);
}

// CONVERT TO PROBABILITY
// log_odd: the log odds of the cell
// converts log odds to a probability
float MyMap::convert_to_probability(float log_odds) 
{
	float probability = 1.0 - (1.0 / (1.0 + exp(log_odds)));
	return probability * 100.0;
}

// DRAW POINT
// is_occupied: bool indicating whether or not 
// the cell detected should be considered occupied
void MyMap::draw_point(int x, int y, bool is_occupied) 
{
	if (y > 0 && y < MAP_HEIGHT && x > 0 && x < MAP_WIDTH) 
	{
		this->map[y][x] = update_log_odds(this->map[y][x], is_occupied);
	}
}

/*
  cuts all lawn in a 15pixel by 15pixel box around lawnmower

  cutting range of 1.5 meter

  written by oliver
 */
void MyMap::cut_area(int x, int y, MyMap otherMap) 
{
  for (int i = -7; i < 8; i++){ // for 15 around y
    for (int j = -7; j < 8; j++){ // for 15 around x
        if ((y+i) > 0 && (y+i) < MAP_HEIGHT && (x+j) > 0 && (x+j) < MAP_WIDTH){ // if within bounds
          if(otherMap.map[y+i][x+j] < 0.5){ // if white on map
            this->map[y+i][x+j] = 0.1; // set to 1, which means it is cut.
          }//end if
        }//end if
    }//end for
  }//end forooooo
}//end function

  /**
     other map expects the map that holds map data, not the map that holds lawn cut data 
  - Oliver 
  */
void MyMap::add_uncut(MyMap otherMap){
  for (int i = 0; i < MAP_HEIGHT; i++){
    for (int j = 0; j < MAP_WIDTH; j++){
      // if white on map and not cut on map_covered
      if( otherMap.map[i][j] < 0.5 &&
	  otherMap.map[i][j] != -1 &&
	  this->map[i][j] != 0.1) {
	
	this->map[i][j] = .9;
      } //end if
    }//end for
  }//end for
}//end function

void MyMap::compare_results(){
  // counter of 0's and 1's in map_covered
  int num_cut = 0;
  float num_uncut = 0.0;
  float total = 1.0;
  int num_unknown = 0;
  
  for (int i = 0; i < MAP_HEIGHT; i++){
    for (int j = 0; j < MAP_WIDTH; j++){	  
      if( this->map[i][j] < 0.5) //.1
          num_cut++;
      if( this->map[i][j] > 0.5) //.9
          num_uncut++;
      if( this->map[i][j] == 0.5)
          num_unknown++;
    }
  }
  total = num_uncut + num_cut;

  std::cout << num_unknown << " spaces unknown\n";
  std::cout << num_uncut << " spaces uncut\n";
  std::cout << num_cut << " spaces cut\n";
  std::cout << total << " spaces total\n";
  std::cout << (num_cut/total) << " coverage by the lawn mower\n";
}

// UPDATE LOG ODDS
// old_odds: prior log odds of the current cell
// is_occupied: bool indicating if the cell in question is occupied or not
float MyMap::update_log_odds(float old_odds, bool is_occupied) 
{
	float new_odds = old_odds;

	if (is_occupied)
        new_odds+= L_OCC - L0;
	else
        new_odds+= L_FREE - L0;
	return new_odds;
}

// DRAW_LINE FUNCTION
// This code was taken from 
// https://stackoverflow.com/questions/10060046/drawing-lines-with-bresenhams-line-algorithm
// and cleaned up and modified a little by me
void MyMap::draw_line(int x1, int y1, int x2, int y2) 
{
 	int x, y, xe, ye, dx, dy, dx1, dy1, px, py;
	float distance;
	float full_distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
 	dx = x2 - x1;
 	dy = y2 - y1;
 	dx1 = fabs(dx);
	dy1 = fabs(dy);
 	px = 2 * dy1 - dx1;
 	py = 2 * dx1 - dy1;

 	if (dy1 <= dx1)
 	{
  		if (dx >= 0)
  		{
   			x = x1;
   			y = y1;
   			xe = x2;
  		}
  		else
 		{
   			x = x2;
   			y = y2;
   			xe = x1;
  		}
		this->draw_point(x, y, false);
  		for (int i = 0; x < xe; i++)
  		{
   			x = x + 1;
			if (px < 0) px = px + 2 * dy1;
   			else	
   			{
    				if ( (dx < 0 && dy < 0) || (dx > 0 && dy > 0) ) y = y + 1;
    				else y = y - 1;
    				px = px + 2 * (dy1 - dx1);
   			}
			this->draw_point(x, y, false);
  		}
 	}
 	else
 	{
  		if (dy >= 0)
  		{
   			x = x1;
   			y = y1;
   			ye = y2;
  		}
  		else
  		{
   			x = x2;
   			y = y2;
   			ye = y1;
  		}
		this->draw_point(x, y, false);
  		for (int i = 0; y < ye; i++)
  		{
   			y = y + 1;

   			if (py <= 0) py=py+2*dx1;
   			else
   			{
    				if ( (dx < 0 && dy < 0) || (dx > 0 && dy > 0) ) x = x + 1;
    				else x = x - 1;
    				py = py + 2 * (dx1 - dy1);
   			}
			this->draw_point(x, y, false);
  		}
 	}
}

// functions added by oliver below


/**
   looks in map for last column of unmowed stuff
   cannot be used if second map has all 0's.
*/
int MyMap::find_unmowed(){

  int next_column = 0;
  
  for (int i = 0; i < MAP_HEIGHT; i++) 
    {
      
      for (int j = 0; j < MAP_WIDTH; j++) 
	{

	  if (map[i][j] == 1 && j < next_column)
	    next_column = j+1;
	}
    } 
}


/**
   marks map_covered with a 1 where ever lawnmower robot has been.
*/
void MyMap::mark_as_mowed(int x, int y){
  map[x][y] = 1;
}
//////////////////////////////////////////////////////////////
