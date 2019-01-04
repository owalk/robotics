# mcl_ws

=====


**Build & Run**
```
cd mcl_ws
rm -r devel build
catkin_make
source devel/setup.bash
```

Now run:
```
roslaunch uml_mcl mcl.launch

rviz

rosrun mower_brain map

```