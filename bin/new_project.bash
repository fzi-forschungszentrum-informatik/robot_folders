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

if [ -z ${SCRIPT_DIR+x} ]; then
    echo "SCRIPT_DIR is unset. Please source the prepare_robot_folders script first."
else
    exit
fi

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
    echo "Usage: $0 <project_name>"
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
cp $template_dir_/setup.sh $project_dir_/setup.sh

if [ $create_ic_workspace_ = true ]; then
    echo "Creating IcWorkspace"
    cd $project_dir_ && git clone git://idsgit.fzi.de/core/ic_workspace.git $ic_workspace_dir_name_ && cd $ic_workspace_dir_name_ && ./IcWorkspace.py grab base -a
    if [ ! -e "$project_dir_/$ic_workspace_dir_name_/IcWorkspace.py" ]; then
        echo "Something went wrong when creating the IcWorkspace"
    fi
fi

if [ $create_ros_workspace_ = true ]; then
    echo "Creating ROS workspace"
    source "/opt/ros/$ros_distro_/setup.$SOURCE_ENDING"
    cd $project_dir_ && mkdir -p "$ros_workspace_dir_name_/src" && cd $ros_workspace_dir_name_/src && catkin_init_workspace .
    if [ ! -e "$project_dir_/$ros_workspace_dir_name_/src/CMakeLists.txt" ]; then
        echo "Something went wrong when creating the ROS workspace"
    fi
fi

echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo "--------------------------------"
echo "Typical next steps are:"
echo ""
echo "# First of all, OPEN A NEW TERMINAL to keep this howto on the screen."
echo ""
echo "# Then, set appropriate environment variables and aliases for your project:"
echo "  source $project_dir_/setup.$SOURCE_ENDING"
echo ""
echo "# This can also be done visually running ce (this also resets the console):"
echo "  ce"
echo ""
echo "# One alias is to easily get into your ic_workspace folder:"
echo "  cdic"
echo ""
echo "# for your new project a basic ic_workspace is already checked out."
echo "# You will have to create your build folder:"
echo "  mkdir build"
echo "  cd build"
echo "  cmake .."
echo "  make -j4 install"
echo ""
echo "# Since you will want to run the last step quite often, there is"
echo "# another alias that can be run from any folder."
echo "# (You will have to rerun ce to register the alias)"
echo "  makeic"
echo ""
echo "# After configuring your catkin_ws, makeros and cdros will work in a similar way."
echo ""
echo "# To configure ROS first manually move into the catkin_ws:"
echo "  cd $project_dir_/catkin_ws"
echo ""
echo "# Then initialize the devel/setup.$SOURCE_ENDING by running catkin_make"
echo "  source /opt/ros/indigo/setup.$SOURCE_ENDING"
echo "  catkin_make"
echo ""
echo "# Afterwards a new ce will also register the ROS aliases:"
echo "  makeros"
echo "  cdros"
echo ""
echo "# Now you are ready to use your ic_workspace with catkin_ws."
echo "# For example you can start by checking out some ic packages:"
echo "  cdic"
echo "  ./IcWorkspace.py grab icl_timesync icl_vision"
echo "  makeic"
echo "  icl_stream image+test:"
echo "--------------------------------"
