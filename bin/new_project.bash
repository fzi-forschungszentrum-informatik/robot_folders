#! /bin/bash
#
# author  Sebastian Klemm <klemm@fzi.de>
# date    2014-11-02
#
# This shell script serves as helper tool for creating
# project directories on a robot.
#
# It currently supports creation of
#  - IcWorkspace
#  - ROS workspace
#----------------------------------------------------------------------


# ==================================================
#   Default Variables
# ==================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CHECKOUT_DIR=$( readlink -e $SCRIPT_DIR/../checkout )


# ==================================================
#   Base configuration
# ==================================================

# Whether to create an ic_worskpace
create_ic_workspace_=true
# Whether to create a ros_workspace
create_ros_workspace_=true
# ROS distro to use
ros_distro_="indigo"
# Name of the ic_workspace directory
ic_workspace_dir_name_="ic_workspace"
# Name of the ros workspace directory
ros_workspace_dir_name_="catkin_ws"

# ==================================================
#   Advanced configuration
# ==================================================

# Where to copy script template(s) from
template_dir_="$CHECKOUT_DIR/example"

project_dir_=$CHECKOUT_DIR/$1

# ==================================================
#   Sanity checks
# ==================================================
if [ -z $1 ]; then
    echo "Usage: ${BASH_SOURCE[0]} <project_name>"
    exit -1
fi

if [ -d $project_dir_ ]; then
    echo "Directory '$project_dir_' already exists, aborting ..."
    exit -1
fi

# ==================================================
#   Do stuff
# ==================================================

# create project directory
mkdir $project_dir_

echo "Copying project setup script"
cp $template_dir_/setup.bash $project_dir_/setup.bash

if [ $create_ic_workspace_ = true ]; then
    echo "Creating IcWorkspace"
    cd $project_dir_ && git clone git://idsgit.fzi.de/core/ic_workspace.git $ic_workspace_dir_name_ && cd $ic_workspace_dir_name_ && ./IcWorkspace.py grab base -a
    if [ ! -e "$project_dir_/$ic_workspace_dir_name_/IcWorkspace.py" ]; then
        echo "Something went wrong when creating the IcWorkspace"
    fi
fi

if [ $create_ros_workspace_ = true ]; then
    echo "Creating ROS workspace"
    source "/opt/ros/$ros_distro_/setup.bash"
    cd $project_dir_ && mkdir -p "$ros_workspace_dir_name_/src" && cd $ros_workspace_dir_name_/src && catkin_init_workspace .
    if [ ! -e "$project_dir_/$ros_workspace_dir_name_/src/CMakeLists.txt" ]; then
        echo "Something went wrong when creating the ROS workspace"
    fi
fi

echo ""
echo ""
echo "--------------------------------"
echo "Typical next steps are:"
echo ""
echo "  source $project_dir_/setup.bash"
echo "  cdic"
echo "  mkdir build"
echo "  cd build"
echo "  cmake .."
echo "  make -j4 install"
echo ""
echo "  source /opt/ros/indigo/setup.bash"
echo "  cdros"
echo "  catkin_make"
echo "  source $project_dir_/setup.bash"
echo ""
echo "  cd $project_dir_/$ic_workspace_dir_name_"
echo "  ./IcWorkspace.py grab icl_timesync"
echo "--------------------------------"
