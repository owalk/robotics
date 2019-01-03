# mcl_ws

=====


**Build & Run**
```
cd mcl_ws
rm -r devel build
catkin_make
source devel/setup.bash
cd src/no_weights/src/raycaster
make clean
make
cd ../../../with_weights/src/raycaster
make clean
make
```

Now run:
```
roslaunch uml_mcl mcl.launch

rviz

rosrun pset6_occupancy_grid_mapping map

```

Note:

there is a zipped file called version_before_partner_integration.zip that contains the code before my partner and I integrated our code. it shows how we changed the class structure to make his edits work with what I wrote.
