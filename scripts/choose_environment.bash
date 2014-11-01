#!/bin/bash

###############################################################
#
# This file is part of the robot_folders repository system.
#
# Don't make any changes in this script. It will affect all the
# available workspaces. Instead use the setup.bash inside your
# workspace to add personal modifications.
#
# If you want to add a new workspace you can start with the 
# setup.bash from the example project.
#
# This script can also be available as an alias (ce) by adding
# this line to your bash_rc:
# alias ce='. ~/scripts/choose_environment.bash'
#
###############################################################


# get checkout dir relative to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
checkout_dir=$SCRIPT_DIR/../checkout

# let the user choose an environment
#
# TODO: auto-search for available workspaces (holding setup.bash or similar) 
# in the checkout_dir
#
# TODO: change this script to a commandline-only version where you can either
# add a commandline parameter directly or choose the desired workspace from 
# a list at the commandline.
#
# TODO: maybe change this script to python
#
workspace_name=`kdialog --menu "Choose a Workspace:" "example" "Example"` 

environment=$checkout_dir/$workspace_name
echo Switching to environment: $environment

echo Running $environment/setup.bash ...
source $environment/setup.bash


# finish
echo complete. 

