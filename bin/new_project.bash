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

# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

usage () {
  cat <<EOF
    usage: $0 options <project_name>

    This script will create a new workspace and checkout a basic ic_workspace
    and catkin_workspace.

    OPTIONS:
    -h  show this message
    -i <true/false> explicitly enable / disable ic_workspace
    -r <true/false> explicitly enable / disable ros_workspace
    -n use ninja for building catkin workspace
EOF
}

# ==================================================
#   Default Variables
# ==================================================

if [ -z ${ROB_FOLDERS_SCRIPT_DIR+x} ]; then
    echo "ROB_FOLDERS_SCRIPT_DIR is unset. Please source the prepare_robot_folders script first."
    exit
fi

CHECKOUT_DIR=$( readlink -e $ROB_FOLDERS_SCRIPT_DIR/../checkout )

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
use_ninja=''

# ==================================================
#   Parse commandline arguments
# ==================================================
while getopts "h?i:r:n" opt; do
  case "$opt" in
   h\?)
     usage
     exit 0
     ;;
   i)  create_ic_workspace_=$OPTARG
     ;;
   r)  create_ros_workspace_=$OPTARG
     ;;
   n) use_ninja="--use-ninja"
     echo "Using ninja for catkin_make"
     ;;
  esac
done

echo "Use ic_workspace: $create_ic_workspace_"
echo "Use ros_workspace: $create_ros_workspace_"

shift $((OPTIND-1))

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
    source "/opt/ros/$ros_distro_/setup.bash"
    cd $project_dir_ && mkdir -p "$ros_workspace_dir_name_/src" && cd $ros_workspace_dir_name_/src && catkin_init_workspace .
    echo "executing catkin_make $use_ninja"
    cd $project_dir_/$ros_workspace_dir_name_ && catkin_make $use_ninja
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
echo "  source $project_dir_/setup.$ROB_FOLDERS_SOURCE_ENDING"
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
echo "# Then initialize the devel/setup.$ROB_FOLDERS_SOURCE_ENDING by running catkin_make"
echo "  source /opt/ros/indigo/setup.$ROB_FOLDERS_SOURCE_ENDING"
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
