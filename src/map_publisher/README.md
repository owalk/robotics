# map_publisher

=====

### **Introduction**
This is for visualizing your map for PS6

For this problem set, you actually don't have to  write your own visualization
software.  ROS has a data type geometry_msgs/OccupancyGrid for this purpose
(http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html).

All you need to do is add geometry_msgs as a dependency (if it's not already),
and import the OccupancyGrid message type, same as the others.

I have created an example Occupancy Grid publisher that creates a map of size
[1200,300] pixels, using a resolution of 5 pixels per meter.  I then
initialize all of the values to unknown and publish the map.  To view the map,
simply ```rosrun rviz rviz``` from the terminal, click the "add" button in the
middle left area, change it to "by topic", and select map.

You may use part or all of this for your own solution.  You are not required
to use our map size or resolutions, and you can still use your own
visualization if you wish, but this one should work pretty well.

- Jordan Alspaw and James Kuczynski
