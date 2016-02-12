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

# set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
checkout_dir=$( readlink -e $SCRIPT_DIR/../checkout )
test -e ${checkout_dir} || { echo "Checkout Directory: ${checkout_dir} does not exist"; exit 0; }
workspace_name="${1}"

MENU=$(find ${checkout_dir}/* -maxdepth 1 -iname "setup.bash" -exec readlink -e {} \; | rev | cut -f 2 -d '/' | rev )

if [ "${workspace_name}" == "shortlist" ]; then
  echo $MENU
  exit
fi

reset

if [ "${workspace_name}" == "" ]; then

  select workspace_name in "CANCEL" ${MENU}; do

    if [ "${workspace_name}" == "" ]; then
      echo "Please select a number..."
    else
      break;
    fi
  done

  [ "${workspace_name}" == "CANCEL" ] && { echo "You chose to cancel"; return 0; }
  [ "${workspace_name}" == "" ] && { echo "You chose an invalid option"; return 0; }

  echo ""
  echo "INFO: You can use \"ce ${workspace_name}\" without any dialog next time!"
  echo ""

fi

environment=$( readlink -e $checkout_dir/$workspace_name)
echo Switching to environment: $environment


echo Running $environment/setup.bash ...
test -e $environment/setup.bash || { echo "$environment/setup.bash does not exist!"; return 0; }
source $environment/setup.bash


# finish
echo complete.
