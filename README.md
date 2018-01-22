[![Build Status](https://travis-ci.org/graspit-simulator/graspit_interface.svg?branch=master)](https://travis-ci.org/graspit-simulator/graspit_interface)

graspit_interface
=================

This plugin exposes a ROS interface for the GraspIt! simulator. The main purpose for writing this plugin was to demonstrate what we believe is the easiest way to expose GraspIt!
functionality as a variety ROS services and action servers. 

Please feel free to use this as a template to write your own bridge between a ros system and GraspIt!.

To see how a client interacts with this interface, check out our python client
[graspit_commander](https://github.com/graspit-simulator/graspit_commander).


GraspIt Setup:
------
```
git clone https://github.com/graspit-simulator/graspit.git
cd graspit
mkdir build
cd build
cmake ..
make -j5
sudo make install
```

You might need to add /usr/local/lib to the loaded library path as in:
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```
You will also want to set the GRASPIT environment variable:
```
export GRASPIT=~/.graspit
```
On Linux, you can add both of these lines to the bottom of your ~/.bashrc

ROS Setup:
------

After running the GraspIt! Setup as explained above, the following details how to setup GraspIt! to work with ROS.

```
//create ros workspace
mkdir -p graspit_ros_ws/src
cd graspit_ros_ws/src

source /opt/ros/indigo/setup.bash
catkin_init_workspace . 

//clone packages
git clone https://github.com/graspit-simulator/graspit_interface.git
git clone https://github.com/graspit-simulator/graspit_commander.git

//build workspace
cd graspit_ros_ws
catkin_make
```


Launching graspit_interface:
-------
```
source devel/setup.bash
roslaunch graspit_interface graspit_interface.launch
```

Then you can view available services and topics provided by the graspit_interface via:
```
rostopic list
rosservice list
```
