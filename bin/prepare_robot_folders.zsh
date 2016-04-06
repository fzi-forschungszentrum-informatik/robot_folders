#! /bin/zsh
#
# author  Felix Mauch <mauch@fzi.de>
# date    2016-04-05
# 
#
# This shell script prepares the bash to execute robot_folder_scripts
# Add this script to your .zshrc:
#
#if [ -f ~/robot_folders/bin/prepare_robot_folders.zsh ]; then
#    source ~/robot_folders/bin/prepare_robot_folders.zsh
#fi
#----------------------------------------------------------------------

# ==================================================
#   Default Variables
# ==================================================

export ROB_FOLDERS_SCRIPT_DIR="$( cd "$( dirname "${(%):-%N}" )" && pwd )"
export ROB_FOLDERS_SOURCE_ENDING=zsh

emulate -R zsh -c 'source "$ROB_FOLDERS_SCRIPT_DIR/prepare_robot_folders.sh"'
# add an alias that sources the choose_environment.sh
alias ce='source '$ROB_FOLDERS_SCRIPT_DIR'/choose_environment.sh'
