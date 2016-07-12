# graspit_interface
ROS interface for GraspIt!.  This repo contains a plugin for GraspIt! that exposes many of the commonly used features within GraspIt! as ros services and action servers.  This makes it easier for developers to interface with GraspIt! programatically without writing their own GraspIt! plugin. 

While it is entirely possible to write your own ros service clients that this plugin exposes, if you're using python, this is already done for you in the graspit_commander package:
https://github.com/CURG/graspit_commander


Setup:
```
cd ~
mkdir graspit_ros_ws
cd graspit_ros_ws
mkdir src
cd src

source /opt/ros/indigo/setup.bash
catkin_init_workspace .

git clone git@github.com:CURG/graspit_interface.git
git clone git@github.com:CURG/graspit_commander.git
git clone git@github.com:graspit-simulator/graspit-ros.git
```
Change GraspIt! branch, this will go away, once this branch is pulled into 
git@github.com:graspit-simulator/graspit.git

```
cd graspit-ros/graspit
rm -rf graspit_source
git clone git@github.com:jvarley/graspit.git graspit_source 
cd graspit_source 
git checkout origin/refactor_src

cd ~/graspit_ros_ws
catkin_make
```

Run:
```
source devel/setup.bash
ipython
from graspit_commander import GraspitCommander
GraspitCommander.*
```
