#! /usb/bin/env sh
#
# author  Ralf Kohlhaas <kohlhaas@fzi.de>
# date    2015-01-23
#
# This shell script prepares the bash to execute robot_folder_scripts
# Add this script to your .bashrc:
#
#if [ -f ~/robot_folders/bin/prepare_robot_folders.bash ]; then
#    . ~/robot_folders/bin/prepare_robot_folders.bash
#fi
#----------------------------------------------------------------------

# ==================================================
#   Default Variables
# ==================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SOURCE_ENDING=bash
source $SCRIPT_DIR/prepare_robot_folders.sh

source $SCRIPT_DIR/bash_completion.sh

# add an alias that sources the choose_environment.bash
alias ce='source '$SCRIPT_DIR'/choose_environment.sh'
