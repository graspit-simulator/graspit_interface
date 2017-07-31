export ROS_DISTRO=indigo

APT='
cmake
libqt4-dev
libqt4-opengl-dev
libqt4-sql-psql
libcoin80-dev
libsoqt4-dev
libblas-dev
liblapack-dev
libqhull-dev 
libeigen3-dev
python-catkin-pkg
python-rosdep
python-wstool
ros-${ROS_DISTRO}-catkin
ros-${ROS_DISTRO}-sensor-msgs
ros-${ROS_DISTRO}-geometry-msgs
ros-${ROS_DISTRO}-tf-conversions
ros-${ROS_DISTRO}-shape-msgs
'

sudo apt-get -qq --yes --force-yes install $APT
