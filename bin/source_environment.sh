#################################################################
#------- This script is part of the robot folders system -------#
#                                                               #
# This script handles sourcing of an environment. Simply        #
# source this file using                                        #
#   source source_environment.sh                                #
# This script will search for an ic_workspace,                  #
# a catkin_workspace and an mca_workspace and call their        #
# respective source files.                                      #
# Afterwards the sourc_local.sh file will be sourced if present.#
#                                                               #
# Please do not make modifications to this file, if you don't   #
# know exactly what you're doing. Use the source_local.sh file  #
# inside the environment_dir which will be sourced afterwards.  #
#                                                               #
# Usually a symlink to this file will be stored in each         #
# environment. So when copying an environment to a location     #
# where no robot_folders are available, remember to copy this   #
# file to the symlink's location.                               #
#                                                               #
# You can either source this symlink directly with the above    #
# command or use the robot_folders change_environment command.  #
#################################################################

# At first, we determine which shell type we're in.
# Note that the $environment_dir will be set from outside, if we called this script using
# robot_folders. Otherwise we assume, that we're manually sourcing the source file inside an
# environment directory.

# zsh
if [ -n "${ZSH_VERSION+1}" ];
then
  if [ -z $environment_dir ]
  then
    # Get the base directory where the install script is located
    environment_dir="$( cd "$( dirname "${(%):-%N}" )" && pwd )"
  fi

  shell_type="zsh"
fi

# bash
if [ -n "${BASH_VERSION+1}" ];
then
  if [ -z $environment_dir ]
  then
    # Get the base directory where the install script is located
    environment_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  fi
  shell_type="bash"
fi


# This is the environment's name, which we will print out later
env_name=$(basename $environment_dir)

# Remove the packages cache from cmake as this creates problems with multiple envs
rm -rf $HOME/.cmake/packages/

# This is basically only relevant when calling the script with an externally defined environment_dir
if [ -d $environment_dir ]; then
  # Run ic_workspace initialization if available
  ic_dir=$environment_dir/ic_workspace
  if [ -f $ic_dir/CMakeLists.txt ]
  then
    export LD_LIBRARY_PATH=$ic_dir/export/lib/:$LD_LIBRARY_PATH
    export PYTHONPATH=$ic_dir/export/lib/python2.7/site-packages:$PYTHONPATH
    export PATH=$ic_dir/export/bin:${PATH}
    export QML_IMPORT_PATH=$ic_dir/export/plugins/qml:$QML_IMPORT_PATH
    export CMAKE_PREFIX_PATH=$ic_dir/export:$CMAKE_PREFIX_PATH
    export IC_MAKER_DIR=$ic_dir/icmaker
    echo "Sourced ic_workspace"
  fi

  # Run ROS initialization if available
  # We run the setup.sh in the catkin_ws folder. Afterwards we can run rosrun, roslaunch etc. with the files in it.
  catkin_dir=$environment_dir/catkin_workspace
  if [ -d $catkin_dir ]
  then
    if [ -f $catkin_dir/devel_isolated/setup.$shell_type ]
    then
      source $catkin_dir/devel_isolated/setup.$shell_type
      echo "Sourced catkin_workspace"
    elif [ -f $catkin_dir/devel/setup.$shell_type ]
    then
      source $catkin_dir/devel/setup.$shell_type

      echo "Sourced catkin_workspace"
    else
      echo "no setup.$shell_type for the ROS workspace found: $catkin_dir/devel/setup.$shell_type. Run catkin_make inside $catkin_dir to create the devel/setup.$shell_type."
    fi
  fi

  # source mca environment
  mca_dir=$environment_dir/mca_workspace
  if [ -d $mca_dir ]
  then
    pushd . &> /dev/null
    cd $mca_dir/build
    source script/mcasetenv
    popd . &> /dev/null
    echo "Sourced mca_workspace"
  fi

  # source local source file
  if [ -f $environment_dir/source_local.sh ];
  then
    source $environment_dir/source_local.sh
  fi

  echo "Environment setup for '${env_name}' done. You now have a sourced environment."
else
  echo "No environment with the given name found!"
fi
