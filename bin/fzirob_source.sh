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

add_fzi_project()
{
  echo "DEPRECATED! DEPRECATED! DEPRECATED! DEPRECATED!"
  echo "The use of this call is deprecated!!!"
  echo "DEPRECATED! DEPRECATED! DEPRECATED! DEPRECATED!"
  echo "Please use the command 'fzirob add_environment' in future"
  echo "DEPRECATED! DEPRECATED! DEPRECATED! DEPRECATED!"
  echo "Executing 'fzirob add_environment' now"
  fzirob add_environment $@
}

# define some other useful aliases
alias cs="fzirob source_most_recent_env"


# These aliases are created after an environment is changed into.
# They are environment-specific but are created for all environments.
# Note: This is not done in the source_envoronment.sh script as the source
# script might be overwritten by the user and then suddenly these aliases
# won't be there anymore. We decided to keep all alias definitions inside
# this file.
env_aliases()
{
    alias kdevsession="kdevelop -s ${ROB_FOLDERS_ACTIVE_ENV}"
    alias qtcreatoric="fzirob cd ic && cd .. && qtcreator ."
}


# Create the fzirob function
#
# Since rob_folders is a python program, it cannot execute commands
# directly on the shell such as 'cd' or setting environment variables.
# For this reason we created this wrapper function that performs
# the shell actions.
fzirob()
{
    if [ $# -ge 1 ]; then
      # if we want to cd to a directory, we need to capture the output
      if [ $1 = "cd" ]; then
          output=$(rob_folders $@)
          echo "$output"
          cd_target=$(echo "$output" | grep "^cd" | tail -n 1 | sed s/cd\ //)
          if [ ! -z "${cd_target// }" ]; then
              cd ${cd_target}
          fi
      else
          rob_folders $@
          if [ $1 = "change_environment" ] || [ $1 = "source_most_recent_env" ]; then
              checkout_dir=$(rob_folders get_checkout_base_dir)
              export ROB_FOLDERS_ACTIVE_ENV=$(cat ${checkout_dir}/.cur_env)
              environment_dir="${checkout_dir}/${ROB_FOLDERS_ACTIVE_ENV}"
              if [ -f ${environment_dir}/setup.sh ]; then
                source ${environment_dir}/setup.sh
              elif [ -f ${environment_dir}/setup.zsh ]; then
                source ${environment_dir}/setup.zsh
              elif [ -f ${environment_dir}/setup.bash ]; then
                source ${environment_dir}/setup.bash
              else
                source ${ROB_FOLDERS_BASE_DIR}/bin/source_environment.sh
              fi
              # declare environment-specific aliases
              env_aliases
          fi
      fi
    else
      rob_folders --help
    fi
}
