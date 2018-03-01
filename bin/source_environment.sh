#################################################################
#------- This script is part of the robot folders system -------#
#                                                               #
# This script handles sourcing of an environment. Simply        #
# source this file using                                        #
#   source source_environment.sh                                #
# This script will search for an ic_workspace,                  #
# a catkin_workspace and an mca_workspace and call their        #
# respective source files.                                      #
# Afterwards the setup_local.sh file will be sourced if present.#
#                                                               #
# Please do not make modifications to this file, if you don't   #
# know exactly what you're doing. Use the setup_local.sh file  #
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
  if [ -z ${ROB_FOLDERS_EMPTY_PATH} ]; then
    # Set the prefix path to the one stored away when starting a new session
    export CMAKE_PREFIX_PATH=${ROB_FOLDERS_EMPTY_CMAKE_PATH}
    export PATH=${ROB_FOLDERS_EMPTY_PATH}
    export LD_LIBRARY_PATH=${ROB_FOLDERS_EMPTY_LD_LIBRARY_PATH}
    export QML_IMPORT_PATH=${ROB_FOLDERS_EMPTY_QML_IMPORT_PATH}
    export PYTHONPATH=${ROB_FOLDERS_EMPTY_PYTHONPATH}
  fi
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
  catkin_dir_long=$environment_dir/catkin_workspace
  catkin_dir_short=$environment_dir/catkin_ws

  # check if catkin_ws is used
  if [ -d $catkin_dir_short ]
  then
    catkin_dir=$catkin_dir_short
  else
    catkin_dir=$catkin_dir_long
  fi

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
      echo "no setup.$shell_type for the ROS workspace found. Sourcing global ROS"
      num_ros_distros=$(find /opt/ros -maxdepth 1 -mindepth 1 -type d | wc -l)
      if [[ $num_ros_distros -gt 1 ]]; then
        echo "Found more than one ros_distribution:"
        echo $(ls /opt/ros/)
        echo "Please insert the distro that should be used:"
        read ros_distro
      else
        ros_distro=$(ls /opt/ros/)
      fi
      setup_file=/opt/ros/$ros_distro/setup.$shell_type
      source $setup_file
      echo "sourced $setup_file"
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
  if [ -f $environment_dir/setup_local.sh ];
  then
    source $environment_dir/setup_local.sh
  elif [ -f $environment_dir/source_local.sh ];  # for backward compatibility
  then
    source $environment_dir/source_local.sh
  fi

  echo "Environment setup for '${env_name}' done. You now have a sourced environment."
else
  echo "No environment with the given name found!"
fi
