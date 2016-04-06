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

if [ -z ${ROB_FOLDERS_SCRIPT_DIR+x} ]; then
  echo "ROB_FOLDERS_SCRIPT_DIR is unset. Please source the prepare_robot_folders script first."
else
  alias cdhome="cd $ROB_FOLDERS_SCRIPT_DIR"

  # enviroment comes from chose_environment.sh
  project_dir=$environment
  # Run ic_workspace initialization if available
  ic_dir=$project_dir/ic_workspace
  if [ -f $ic_dir/CMakeLists.txt ]
  then
    echo configuring ICL workspace inside $ic_dir ...

    export LD_LIBRARY_PATH=$ic_dir/export/lib/:$LD_LIBRARY_PATH
    export PYTHONPATH=$ic_dir/export/lib/python2.7/site-packages:$PYTHONPATH
    export PATH=$ic_dir/export/bin:${PATH}
    export QML_IMPORT_PATH=$ic_dir/export/plugins/qml:$QML_IMPORT_PATH
    export CMAKE_PREFIX_PATH=$ic_dir/export
    export IC_MAKER_DIR=$ic_dir/icmaker

    export LC_ALL=C

    alias cdic="cd $ic_dir"

    # qtcreatoric will start the qtcreator and tries to load a
    # session matching the name of the project
    alias qtcreatoric="(cd $ic_dir/.. && qtcreator .)"
    alias kdevsession="kdevelop -s ${ROB_FOLDERS_PROJECT_NAME}"
    
    # add makeic according to build environment

    # ninja
    if [ -f $ic_dir/build/build.ninja ]
    then
      alias makeic="(cd $ic_dir/build && ninja install)"
    fi

    # make
    if [ -f $ic_dir/build/Makefile ]
    then
      alias makeic="(cd $ic_dir/build && make -j4 install)"
    fi

    # git shortcuts
    alias statusic="pushd . &> /dev/null && cd $ic_dir && ./IcWorkspace.py status; popd &> /dev/null "
    alias fetchic="pushd . &> /dev/null  && cd $ic_dir && ./IcWorkspace.py fetch;  popd &> /dev/null"
    alias pullic="pushd . &> /dev/null   && cd $ic_dir && ./IcWorkspace.py pull;   popd &> /dev/null"
    alias pushic="pushd . &> /dev/null   && cd $ic_dir && ./IcWorkspace.py push;   popd &> /dev/null"


  else
    echo no ICL workspace found inside $ic_dir.
  fi


  # Run ROS initialization if available
  # We run the setup.sh in the catkin_ws folder. Afterwards we can run rosrun, roslaunch etc. with the files in it.
  catkin_dir=$project_dir/catkin_ws
  if [ -f $catkin_dir/devel/setup.$ROB_FOLDERS_SOURCE_ENDING ]
  then
    echo configuring ROS workspace via $catkin_dir/devel/setup.$ROB_FOLDERS_SOURCE_ENDING ...

    source $catkin_dir/devel/setup.$ROB_FOLDERS_SOURCE_ENDING
    alias cdros="cd $catkin_dir"
    alias makeros="(cd $catkin_dir && catkin_make)"
  else
    echo no setup.$ROB_FOLDERS_SOURCE_ENDING for the ROS workspace found: $catkin_dir/devel/setup.$ROB_FOLDERS_SOURCE_ENDING. Run catkin_make inside $catkin_dir to create the devel/setup.$ROB_FOLDERS_SOURCE_ENDING.
  fi

  # git shortcuts
  alias statusros="pushd . &> /dev/null && cd $catkin_dir/src &&
  for dir in *
  do
    if [ -d '$dir' ];
    then
      cd '$dir' &> /dev/null
      echo 'Package $dir:'
      git status;
      printf '\n'
      cd .. &> /dev/null
    fi
  done;
  popd &> /dev/null"

  alias fetchros="pushd . &> /dev/null && cd $catkin_dir/src &&
  for dir in *
  do
    if [ -d '$dir' ];
    then
      cd '$dir' &> /dev/null
      echo 'Package $dir:'
      git fetch;
      printf '\n'
      cd .. &> /dev/null
    fi
  done;
  popd &> /dev/null"


  alias pullros="pushd . &> /dev/null && cd $catkin_dir/src &&
  for dir in *
  do
    if [ -d '$dir' ];
    then
      cd '$dir' &> /dev/null
      echo 'Package $dir:'
      git pull;
      printf '\n'
      cd .. &> /dev/null
    fi
  done;
  popd &> /dev/null"


  alias pushros="pushd . &> /dev/null && cd $catkin_dir/src &&
  for dir in *
  do
    if [ -d '$dir' ];
    then
      cd '$dir' &> /dev/null
      echo 'Package $dir:'
      git push;
      printf '\n'
      cd .. &> /dev/null
    fi
  done;
  popd &> /dev/null"


  # Special initialization for this workspace config

  ## ADD YOUR CHANGES HERE!
fi
