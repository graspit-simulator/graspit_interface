graspit_interface
=================

This plugin exposes a ROS interface for the GraspIt! simulator. Our main purpose for writing
this plugin was to give GraspIt! more exposure to the ROS community and make it easier to
get started planning grasps with GraspIt! You can interact with this interface through a
variety of ROS services and action servers.

If you have any specific features you'd like to see, let us know by submitting a new issue, or feel free to submit a pull request!

To see how a client interacts with this interface, check out our python
[graspit_commander](https://github.com/CURG/graspit_commander).


Setup:
------
```
//create ros workspace
mkdir -p graspit_ros_ws/src
cd graspit_ros_ws/src

source /opt/ros/indigo/setup.bash
catkin_init_workspace . 

//clone packages
git clone git@github.com:CURG/graspit_interface.git
git clone git@github.com:CURG/graspit_commander.git
git clone git@github.com:graspit-simulator/graspit-ros.git --recursive

//build workspace
cd graspit_ros_ws
catkin_make
```


Run the interface!:
-------
```
source devel/setup.bash
roslaunch graspit_interface graspit_interface.launch
```
