
# zsh
if [ -n "`$SHELL -c 'echo $ZSH_VERSION'`" ];
then
# Get the base directory where the install script is located
  ROB_FOLDERS_SCRIPT_DIR="$( cd "$( dirname "${(%):-%N}" )" && pwd )"
# autoload bashcompinit if using zsh
  autoload -U bashcompinit && bashcompinit
fi

# bash
if [ -n "`$SHELL -c 'echo $BASH_VERSION'`" ];
then
# Get the base directory where the install script is located
  ROB_FOLDERS_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
fi

# robot_folders setup
export PATH=$PATH:${ROB_FOLDERS_SCRIPT_DIR}/venv/bin

# sourcing alias
source ${ROB_FOLDERS_SCRIPT_DIR}/rob_folders-complete.sh

fzrob()
{
    # if we want to cd to a directory, we need to capture the output
    if [ $1 = "cd" ]; then
        output=$(rob_folders $@)
        echo $output
        cd_target=$(echo $output | grep ^cd | tail -n 1 | sed s/cd\ //)
        cd ${cd_target}
    else
        rob_folders $@
        if [ $1 = "change_environment" ]; then
            source ${ROB_FOLDERS_SCRIPT_DIR}/checkout/.source_cur_env
        fi
    fi
}
