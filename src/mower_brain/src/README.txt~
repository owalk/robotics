////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Email: Victoria_Albanese@student.uml.edu
// Date: November 7, 2017
// Filename: README.txt
//
// Description: This file is a discussion of pset6
//
////////////////////////////////////////////////////////////////////////////////

A 300 to 500 word discussion about what was interesting about the assignment.
Include as part of the discussion:

• Challenges you encountered (and how you solved them)

	The main challenge was really just figuring out conceptually how to 
	solve this problem.  I could figure out how to get the robot to publish 
	its own path to the map easily, and then the endpoints of the sensors 
	from there, but this representation of the world was very spotty.  
	Figuring out that the program had to keep track of all of the points 
	between it and the endpoint of the sensor was overwhelming at first 
	because the grid is pixelated and lines are not.  However, it then 
	occured to me that this is how lines must be rendered when you draw 
	them on a computer because a computer screen is pixelated, so then I 
	simply found an existing line drawing program and used that.  

	Another hurdle was figuring out how to apply all of the math we learned 
	in class to the program.  I ended up finding some really useful videos 
	online that helped improve my understanding, and then having Jordan 
	takeover class on Thursday to explain things also helped immensely.


• Representation of the world map
	
	I used the representation of the world given by Jordan to us, and 
	copied a lot of the code he gave us to use when publishing the 
	occupancy grid.  The only thing I changed was to reduce the 
	granularity of the map so that the height is 150 pixels, the width 
	is 600 pixels; each pixel represents 10cm.


• Performance of your implementation

	My implementation is nicely efficient because I am not going through 
	every cell in the map to see if the cell is on the narrow cone of the 
	laser beam.  Instead, I use a line drawing algorithm to draw a line 
	from the robot's location to the sensor endpoint.  This whole line is 
	treated as free space, using the functions detailed allow to assign 
	actual values to those cells.  If the endpoint of the sensor is less 
	than 5 (meaning that there is an obstacle there), the endpoint, and 3
	points adjacent to it (maing a 2x2 square) are all treated as occupied 
	space.  This was my way of creating a "buffer" space when an object is 
	detected.

	Since the original version of the algorithm much touch every one of the 
	tens of thousands of cells every time an update must be mad, and mine 
	only touches the ones that need touching, my algorithm is far more 
	efficient than the book's solution, while still providing the same 
	functionality.


• Code excerpt accomplishing the probabilistic updates of the map

	This is the function which updates the log odds of the map. 
	Algorithmically, I draw the 5 lines representing the laser beams, 
	and when this happens, we update the map, each cell of which has
	the log odds of the occupation of that cell.

	float MyMap::update_log_odds(float old_odds, bool is_occupied) 
	{
		float new_odds = old_odds;			// L0 = 0.2
		if (is_occupied) new_odds+= L_OCC - L0;		// L_OCC = 0.0
		else new_odds+= L_FREE - L0;			// L_FREE = 1.0
		return new_odds;
	}

	As the map is published, the log odds are converted to probabilities.  
	This version of the map is not saved, merely published.  The function 
	which does this converting is shown below.

	float MyMap::convert_to_probability(float log_odds) 
	{
		float probability = 1.0 - (1.0 / (1.0 + exp(log_odds)));
		return probability * 100.0;
	}


• Additionally

	I included an image of the map created by my program having been run for 
	about a half hour.  Two or three passes are needed to really fill out the 
	map reasonably well, but I just figured that I would include this image 
	because it takes a lot of time to run everyone's program that long, and I 
	wanted you to be able to see a more refined version of the map without 
	needing to wait a half hour.  


////////////////////////////////////////////////////////////////////////////////
