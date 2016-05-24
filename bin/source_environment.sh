#!/bin/sh

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

# enviroment comes from chose_environment.sh

if [ -n "${ROB_FOLDERS_ACTIVE_ENV+1}" ];
then
    # Remove the packages cache from cmake as this creates problems with multiple envs
    rm -rf $HOME/.cmake/packages/
    environment_dir="$ROB_FOLDERS_BASE_DIR/checkout/${ROB_FOLDERS_ACTIVE_ENV}"
    if [ -d $environment_dir ]; then

        shell_type="bash"
        if [ -n "${ZSH_VERSION+1}" ];
        then
            shell_type="zsh"
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
        fi

        # Run ROS initialization if available
        # We run the setup.sh in the catkin_ws folder. Afterwards we can run rosrun, roslaunch etc. with the files in it.
        catkin_dir=$environment_dir/catkin_workspace
        if [ -d $catkin_dir ]
        then
            if [ -f $catkin_dir/devel/setup.$shell_type ]
            then
                source $catkin_dir/devel/setup.$shell_type
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
        fi

    echo "Environment setup for '${ROB_FOLDERS_ACTIVE_ENV}' done. You now have a sourced environment."

    else
        echo "No environment with the given name found!"
    fi
else
    echo "Have not found any active environment. Please call the change_environment or source_active_environment command first!"
fi
