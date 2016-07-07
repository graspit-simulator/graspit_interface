# graspit_interface
ROS interface for GraspIt!

Setup:
'''
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
cd graspit-ros/graspit
rm -rf graspit_source
git clone git@github.com:jvarley/graspit.git graspit_source 
cd graspit_source 
checkout origin/refactor_src

cd ~/graspit_ros_ws
catkin_make
'''

Run:
'''
source devel/setup.bash
ipython
from graspit_commander import GraspitCommander
GraspitCommander.*
'''
