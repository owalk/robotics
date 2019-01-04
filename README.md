# robotics_ws

=====

## Intro

Hi all, this project is a continuation of my work from my mobile robotics 2 class with Holly Yanco and the University of Masschusetts, lowell.
In the project I worked with Wei Li to create an autonomous lawn mower simulation. To extend the work that I got done, I'm going to add in
some path planning logic so that my lawnmower is not mindlessly wondering around and cutting grass but instead actually drives towards uncut grass.
The PDF included in the repository gives an overview of the state of the project when I submitted it for my class.

If you are a random person on the internet and would like to help out, please do! Feel free to email me at oliverw56@gmail.com


## setup enviroment

Install ROS Kinetic with the instructions on ros.org

http://wiki.ros.org/kinetic/Installation

ros kinetic supports Ubuntu 16, Xenial which is what I am running. Figure it out if you have a different system.

in the installation get ros-kinetic-desktop-full

get the following dependencies

sudo apt-get install libcv-dev
sudo apt-get install libcvaux-dev libhighgui-dev
sudo apt-get install python-numpy python-opengl
sudo apt-get install swig

clone my entire work space with

git clone https://github.com/owalk/robotics.git

my work space will have the file paths for my virtual machine. to fix them, delete the top level directories build/ and devel/

rm -r devel/ build/

remake the workspace with

catkin_make

you should see those directories you deleted again.



## running the code

**Build & Run**
```
cd robotics_ws
catkin_make
source devel/setup.bash
```

Now run:
```
roslaunch uml_mcl mcl.launch

rviz

rosrun mower_brain map

for each process that you start, make sure you have source setup.bash in that terminal.

you should always have the mcl.launch running before you run the mower_brain node.
```

