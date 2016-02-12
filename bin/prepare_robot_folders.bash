#! /bin/bash
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
CHECKOUT_DIR=$( readlink -e $SCRIPT_DIR/../checkout )

# add ~/bin to PATH
export PATH=$SCRIPT_DIR:$PATH

# add an alias that sources the choose_environment.bash
alias ce='source '$SCRIPT_DIR'/choose_environment.bash'

# delete the .cmake folder since it is not working with mutliple workspaces
rm -rf $HOME/.cmake/packages/*

source $SCRIPT_DIR/bash_completion.sh
