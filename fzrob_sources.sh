#!/bin/bash

# Get the base directory where the install script is located
ROB_FOLDERS_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# autoload bashcompinit if using zsh 
if [ -n "`$SHELL -c 'echo $ZSH_VERSION'`" ];
then
  autoload -U bashcompinit && bashcompinit
fi

# robot_folders setup
export PATH=$PATH:${ROB_FOLDERS_SCRIPT_DIR}/venv/bin
source ${ROB_FOLDERS_SCRIPT_DIR}/fzrob-complete.sh
