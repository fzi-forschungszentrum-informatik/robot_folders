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

if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi

export ROB_FOLDERS_EMPTY_CMAKE_PATH=${CMAKE_PREFIX_PATH}
export ROB_FOLDERS_EMPTY_PATH=${PATH}
export ROB_FOLDERS_EMPTY_LD_LIBRARY_PATH=${LD_LIBRARY_PATH}
export ROB_FOLDERS_EMPTY_QML_IMPORT_PATH=${QML_IMPORT_PATH}
export ROB_FOLDERS_EMPTY_PYTHONPATH=${PYTHONPATH}

if [ ! -z "${ROB_FOLDERS_EMPTY_CMAKE_PATH}" ] && [ -z $ROB_FOLDERS_IGNORE_CMAKE_PREFIX_PATH ]
then
  echo "WARNING! WARNING! WARNING!"
  echo "Your CMAKE_PREFIX_PATH is not empty. This probably means that you explicitly set your
CMAKE_PREFIX_PATH to include some custom directory. If you want to keep this that way you can
export the environment variable ROB_FOLDERS_IGNORE_CMAKE_PREFIX_PATH to some arbitrary value to
suppress this warning. e.g.

    export ROB_FOLDERS_IGNORE_CMAKE_PREFIX_PATH=\":-)\"
    source /home/mauch/robot_folders/bin/fzirob_source.sh
"
  echo "By, the way your CMAKE_PREFIX_PATH is: \"${CMAKE_PREFIX_PATH}\""
  echo "WARNING! WARNING! WARNING! END."
fi


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

# These aliases are created after an environment is changed into.
# They are environment-specific but are created for all environments.
# Note: This is not done in the source_environment.sh script as the source
# script might be overwritten by the user and then suddenly these aliases
# won't be there anymore. We decided to keep all alias definitions inside
# this file.
env_aliases()
{
  alias kdevsession="kdevelop -s ${ROB_FOLDERS_ACTIVE_ENV}"
  alias qtcreatoric="fzirob cd ic && cd .. && qtcreator ."
}

reset_environment()
{
  export CMAKE_PREFIX_PATH=${ROB_FOLDERS_EMPTY_CMAKE_PATH}
  export PATH=${ROB_FOLDERS_EMPTY_PATH}
  export LD_LIBRARY_PATH=${ROB_FOLDERS_EMPTY_LD_LIBRARY_PATH}
  export QML_IMPORT_PATH=${ROB_FOLDERS_EMPTY_QML_IMPORT_PATH}
  export PYTHONPATH=${ROB_FOLDERS_EMPTY_PYTHONPATH}
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
      if [ $1 = "add_environment" ] && [ "$2" != "--help" ]; then
        echo "Resetting environment before adding new one."
        reset_environment
      fi
      rob_folders $@
      if [ $? -eq 0 ]; then
        if [ $1 = "change_environment" ] && [ "$2" != "--help"  ]; then
          checkout_dir=$(rob_folders get_checkout_base_dir)
          if [ -f ${checkout_dir}/.cur_env ]; then
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
          else
            echo "Could not change environment"
          fi
        fi
      fi
    fi
  else
    rob_folders --help
  fi
}
