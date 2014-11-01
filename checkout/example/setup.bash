#!/bin/bash

###############################################################
#
# This file is part of the robot_folders repository system.
#
# You can copy this file to your personal workspace checkout.
# It should work out of the box for many ic_workspace with 
# catkin_ws configurations. You can also add special commands 
# you want to run. There is a place already prepared at the 
# end of this script.
#
###############################################################


# get this script's directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


# Run ic_workspace initialization if available
ic_dir=$SCRIPT_DIR/ic_workspace
if [ -f $ic_dir/CMakeLists.txt ]
then
  echo configuring ICL workspace inside $ic_dir ...

  export LD_LIBRARY_PATH=$ic_dir/export/lib/:$LD_LIBRARY_PATH
  export PATH=$ic_dir/export/bin:${PATH}
  export QML_IMPORT_PATH=$ic_dir/export/plugins/qml:$QML_IMPORT_PATH
  export CMAKE_PREFIX_PATH=$ic_dir/export
  export IC_MAKER_DIR=$ic_dir/icmaker

  export LC_ALL=C

  alias cdic='cd $ic_dir'
else
  echo no ICL workspace found inside $ic_dir.
fi


# Run ROS initialization if available
# We run the setup.bash in the catkin_ws folder. Afterwards we can run rosrun, roslaunch etc. with the files in it.
catkin_dir=$SCRIPT_DIR/catkin_ws
if [ -f $catkin_dir/devel/setup.bash ]
then
  echo configuring ROS workspace via $catkin_dir/devel/setup.bash ...

  source $catkin_dir/devel/setup.bash
  alias cdros='cd $catkin_dir'
else
  echo no ROS workspace found inside $catkin_dir.
fi


# Special initialization for this workspace config

## ADD YOUR CHANGES HERE!



