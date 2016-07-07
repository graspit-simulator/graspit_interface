#!/bin/bash


if env | grep -q ^GRASPIT=
then
    echo "Using GRASPIT=" $GRASPIT
else
    export GRASPIT=$(rospack find graspit)/graspit_source
fi

export GRASPIT_PLUGIN_DIR=$(dirname $(catkin_find libgraspit_interface.so))

rosrun graspit graspit_simulator -p libgraspit_interface
