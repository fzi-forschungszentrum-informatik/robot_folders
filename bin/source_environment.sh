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
  if [ -n ${ROB_FOLDERS_EMPTY_PATH} ]; then
    # Set the prefix path to the one stored away when starting a new session
    export CMAKE_PREFIX_PATH=${ROB_FOLDERS_EMPTY_CMAKE_PATH}
    export PATH=${ROB_FOLDERS_EMPTY_PATH}
    export LD_LIBRARY_PATH=${ROB_FOLDERS_EMPTY_LD_LIBRARY_PATH}
    export QML_IMPORT_PATH=${ROB_FOLDERS_EMPTY_QML_IMPORT_PATH}
    export PYTHONPATH=${ROB_FOLDERS_EMPTY_PYTHONPATH}
  fi

  # It is important to source the catkin_ws first, as it will remove non-existing paths from the
  # environment (e.g. the export-folder of an ic_ws when not built yet)
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
    if [ -d $catkin_dir/.catkin_tools ]
    then
      SETUP_FILE=$(catkin locate --workspace $catkin_dir -d)/setup.$shell_type
      echo "Sourcing $SETUP_FILE"
      source $SETUP_FILE
    elif [ -f $catkin_dir/devel_isolated/setup.$shell_type ]
    then
      source $catkin_dir/devel_isolated/setup.$shell_type
      echo "Sourced catkin_workspace"
    elif [ -f $catkin_dir/devel/setup.$shell_type ]
    then
      source $catkin_dir/devel/setup.$shell_type

      echo "Sourced catkin_workspace"

    elif [ -f $catkin_dir/install/setup.$shell_type ]
    then 
      echo "Only found installed workspace. Sourcing $catkin_dir/install/setup.$shell_type"
      source $catkin_dir/install/setup.$shell_type
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

  colcon_dir=$environment_dir/colcon_ws
  if [ -d $colcon_dir ]
  then
    if [ -f $colcon_dir/install/local_setup.$shell_type ]
    then
      source $colcon_dir/install/local_setup.$shell_type
      echo "Sourced colcon workspace"
    fi
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

  # Run ic_workspace initialization if available
  ic_dir=$environment_dir/ic_workspace
  if [ -d $ic_dir ]
  then
    if [[ ! "$LD_LIBRARY_PATH" =~ "$ic_dir/export/lib" ]]; then
      export LD_LIBRARY_PATH=$ic_dir/export/lib/:$LD_LIBRARY_PATH
    fi
    if [[ ! "$PYTHONPATH" =~ "$ic_dir/export/lib/python2.7/site-packages" ]]; then
      export PYTHONPATH=$ic_dir/export/lib/python2.7/site-packages:$PYTHONPATH
    fi
    if [[ ! "$PATH" =~ "$ic_dir/export/bin" ]]; then
      export PATH=$ic_dir/export/bin:${PATH}
    fi
    if [[ ! "$QML_IMPORT_PATH" =~ "$ic_dir/export/plugins/qml" ]]; then
      export QML_IMPORT_PATH=$ic_dir/export/plugins/qml:$QML_IMPORT_PATH
    fi
    if [[ ! "$CMAKE_PREFIX_PATH" =~ "$ic_dir/export/" ]]; then
      export CMAKE_PREFIX_PATH=$ic_dir/export:$CMAKE_PREFIX_PATH
    fi
    export IC_MAKER_DIR=$ic_dir/icmaker
    echo "Sourced ic_workspace from $ic_dir"
  fi

  # source misc_ws environment if existing.
  misc_ws_dir=$environment_dir/misc_ws
  if [ -d $misc_ws_dir ]
  then
    if [[ ! "$LD_LIBRARY_PATH" =~ "$misc_ws_dir/export/lib" ]]; then
      export LD_LIBRARY_PATH=$misc_ws_dir/export/lib/:$LD_LIBRARY_PATH
    fi
    if [[ ! "$PATH" =~ "$misc_ws_dir/export/bin" ]]; then
      export PATH=$misc_ws_dir/export/bin:${PATH}
    fi
    if [[ ! "$CMAKE_PREFIX_PATH" =~ "$misc_ws_dir/export/" ]]; then
      export CMAKE_PREFIX_PATH=$misc_ws_dir/export:$CMAKE_PREFIX_PATH
    fi
    echo "Sourced misc_ws workspace from $misc_ws_dir"
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
