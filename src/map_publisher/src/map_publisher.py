#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header

if __name__ == '__main__':
	rospy.init_node("map_publisher")
	pub = rospy.Publisher("/map",OccupancyGrid, queue_size=10) #I create my publisher
	rate = rospy.Rate(10)
	try:
		rospy.delete_param('/use_sim_time') #Rate.sleep wont work unless you remove this param, the simulator does not appear to publish to /clock
	except:
		print("No parameter to delete")

	mymap = OccupancyGrid( #create my message 2400 pixels wide by 600 pixels long, at 0.05 meters per pixel, AKA 120 meters x 30 meters
		header = Header(seq=0, stamp = rospy.Time.now(), frame_id="map"), 
		info = MapMetaData(width=2400, height=600, resolution=0.05,map_load_time=rospy.Time.now()))

	#As per http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html
	#The data is in row-major order, starting witht he 0,0 origin.
	#Probabilities are in a range from [0,100], with -1 being 0
	for i in range(0,600*2400): 
		mymap.data.append(-1.0) 

	while not rospy.is_shutdown():
		pub.publish(mymap)
		rate.sleep()
