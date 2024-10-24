##
## Copyright (c) 2024 FZI Forschungszentrum Informatik
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
## THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
## THE SOFTWARE.
##

#################################################################
#------- This script is part of the robot folders system -------#
#                                                               #
# This script handles sourcing of an environment. Simply        #
# source this file using                                        #
#   source source_environment.sh                                #
# This script will search for a workspace and call their        #
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

  export shell_type="zsh"
fi

# bash
if [ -n "${BASH_VERSION+1}" ];
then
  if [ -z $environment_dir ]
  then
    # Get the base directory where the install script is located
    environment_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  fi
  export shell_type="bash"
fi

if [ -z ${rob_folders_overlay+x} ]; then
  echo "Sourcing environment '$environment_dir'"
  export ROB_FOLDERS_ROOT_ENV=$environment_dir
fi

# This is the environment's name, which we will print out later
env_name=$(basename $environment_dir)

# Remove the packages cache from cmake as this creates problems with multiple envs
rm -rf $HOME/.cmake/packages/

function source_underlay() {
        # cache current recursion step
        local this_env_dir="$environment_dir"
        local this_env_name="$env_name"
        export ROB_FOLDERS_IS_UNDERLAY=true

        export rob_folders_overlay="$this_env_dir"
        echo "Sourcing underlay $1 overlaying $rob_folders_overlay"
        export environment_dir="$1"
        if [ -f ${environment_dir}/setup.sh ]; then
          source ${environment_dir}/setup.sh
        elif [ -f ${environment_dir}/setup.zsh ]; then
          source ${environment_dir}/setup.zsh
        elif [ -f ${environment_dir}/setup.bash ]; then
          source ${environment_dir}/setup.bash
        else
          source ${ROB_FOLDERS_BASE_DIR}/bin/source_environment.sh
        fi
        # reset things
        export environment_dir=$this_env_dir
        export env_name=$this_env_name

}

function source_underlays() {
  local overlay=$1
  local underlay_file="$overlay/underlays.txt"

  if [ -e $underlay_file ]; then
    while read underlay; do
      if [ ! -z "$underlay" ]; then
        source_underlay $underlay
      fi
    done < "$underlay_file"
  fi

}

# This is basically only relevant when calling the script with an externally defined environment_dir
if [ -d $environment_dir ]; then
  if [ -n ${ROB_FOLDERS_EMPTY_PATH} ]; then
    if [ -z "$rob_folders_overlay" ]; then
      # Set the prefix path to the one stored away when starting a new session
      export CMAKE_PREFIX_PATH=${ROB_FOLDERS_EMPTY_CMAKE_PATH}
      export PATH=${ROB_FOLDERS_EMPTY_PATH}
      export LD_LIBRARY_PATH=${ROB_FOLDERS_EMPTY_LD_LIBRARY_PATH}
      export QML_IMPORT_PATH=${ROB_FOLDERS_EMPTY_QML_IMPORT_PATH}
      export PYTHONPATH=${ROB_FOLDERS_EMPTY_PYTHONPATH}
      export AMENT_PREFIX_PATH=${ROB_FOLDERS_EMPTY_AMENT_PREFIX_PATH}
      export COLCON_PREFIX_PATH=${ROB_FOLDERS_EMPTY_COLCON_PREFIX_PATH}
    fi
  fi

  source_underlays $environment_dir


  # It is important to source the catkin_ws first, as it will remove non-existing paths from the
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
      echo "no setup.$shell_type for the catkin workspace found. Sourcing global ROS"
      num_ros_distros=$(find /opt/ros -maxdepth 1 -mindepth 1 -type d | wc -l)
      if [[ $num_ros_distros -eq 0 ]]; then
        echo "No ROS installation found in /opt/ros. Assuming you take care about your ROS setup otherwise."
      else
        if [[ $num_ros_distros -gt 1 ]]; then
          echo "Found more than one ros_distribution:"
          ls /opt/ros/
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
  fi

  colcon_dir_long=$environment_dir/colcon_workspace
  colcon_dir_short=$environment_dir/colcon_ws
  colcon_dir_dev_ws=$environment_dir/dev_ws

  # check if colcon_ws is used
  if [ -d $colcon_dir_short ]
  then
    colcon_dir=$colcon_dir_short
  elif [ -d $colcon_dir_long ]
  then
    colcon_dir=$colcon_dir_long
  else
    colcon_dir=$colcon_dir_dev_ws
  fi

  if [ -d $colcon_dir ]
  then
    if [ -f $colcon_dir/install/local_setup.$shell_type ]
    then
      if [ "$environment_dir" = "$ROB_FOLDERS_ROOT_ENV" ]; then
        ros2_version=$(grep "COLCON_CURRENT_PREFIX=\"/opt/ros" $colcon_dir/install/setup.sh | cut -d '"' -f2)
        echo "Sourcing ${ros2_version}/setup.${shell_type}"
        source "${ros2_version}/setup.${shell_type}"
      fi
      source $colcon_dir/install/local_setup.$shell_type
      echo "Sourced colcon workspace $colcon_dir"
    else
      echo "no setup.$shell_type for the colcon workspace found. Sourcing global ROS2"
      num_ros_distros=$(find /opt/ros -maxdepth 1 -mindepth 1 -type d | wc -l)
      if [[ $num_ros_distros -eq 0 ]]; then
        echo "No ROS installation found in /opt/ros. Assuming you take care about your ROS setup otherwise."
      else
        if [[ $num_ros_distros -gt 1 ]]; then
          echo "Found more than one ros_distribution:"
          ls /opt/ros/
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

  # source local source file
  if [ -f $environment_dir/setup_local.sh ];
  then
    source $environment_dir/setup_local.sh
  elif [ -f $environment_dir/source_local.sh ];  # for backward compatibility
  then
    source $environment_dir/source_local.sh
  fi

  if [ "$environment_dir" = "$ROB_FOLDERS_ROOT_ENV" ]; then
    echo "Environment setup for '${env_name}' done. You now have a sourced environment."
  #else
    #echo "Sourced underlay '${env_name}'"
  fi
else
  echo "No environment with the given name found!"
fi
