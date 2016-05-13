
# zsh
if [ -n "${ZSH_VERSION+1}" ];
then
# Get the base directory where the install script is located
  export ROB_FOLDERS_BASE_DIR="$( cd "$( dirname "${(%):-%N}" )/.." && pwd )"
# autoload bashcompinit if using zsh
  autoload -U bashcompinit && bashcompinit
fi

# bash
if [ -n "${BASH_VERSION+1}" ];
then
# Get the base directory where the install script is located
  export ROB_FOLDERS_BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
fi

export PATH=$PATH:${ROB_FOLDERS_BASE_DIR}/.robot_folders/venv/bin

# sourcing alias
source ${ROB_FOLDERS_BASE_DIR}/bin/rob_folders-complete.sh

# define some legacy aliases from old robot_folders
alias ce="fzirob change_environment"
alias cdros="fzirob cd ros"
alias cdic="fzirob cd ic"
alias cdmca="fzirob cd mca"
alias cdhome="fzirob cd"
alias makeic="fzirob make ic"
alias makeros="fzirob make ros"

# define some other useful aliases
alias cs="fzirob source_most_recent_env"

env_aliases()
{
    alias kdevsession="kdevelop -s ${ROB_FOLDERS_ACTIVE_ENV}"
    alias qtcreatoric="fzirob cd ic && qtcreator ."
}

fzirob()
{
    # if we want to cd to a directory, we need to capture the output
    if [ $1 = "cd" ]; then
        output=$(rob_folders $@)
        echo $output
        cd_target=$(echo $output | grep "^cd" | tail -n 1 | sed s/cd\ //)
        cd ${cd_target}
    else
        rob_folders $@
        if [ $1 = "change_environment" ] || [ $1 = "source_most_recent_env" ]; then
            export ROB_FOLDERS_ACTIVE_ENV=$(cat ${ROB_FOLDERS_BASE_DIR}/checkout/.cur_env)
            source ${ROB_FOLDERS_BASE_DIR}/bin/source_environment.sh
            # declare environment-specific aliases
            env_aliases
        fi
    fi
}
