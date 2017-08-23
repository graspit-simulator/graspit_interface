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
ros-indigo-catkin
ros-indigo-sensor-msgs
ros-indigo-geometry-msgs
ros-indigo-tf-conversions
ros-indigo-shape-msgs
'

sudo apt-get -qq --yes --force-yes install $APT
