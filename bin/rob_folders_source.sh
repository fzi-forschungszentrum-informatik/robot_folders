##
## Copyright (c) 2024 FZI Forschungszentrum Informatik
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
## THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
## THE SOFTWARE.
##

# zsh
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi
my_source="source"
if [ -n "${ZSH_VERSION+1}" ];
then
  # Get the base directory where the install script is located
  export ROB_FOLDERS_BASE_DIR="$( cd "$( dirname "${(%):-%N}" )/.." && pwd )"
  my_source=$(rob_folders_get_source_command.py -s zsh)
fi

# bash
if [ -n "${BASH_VERSION+1}" ];
then
  # Get the base directory where the install script is located
  export ROB_FOLDERS_BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
  my_source=$(rob_folders_get_source_command.py -s bash)
fi
eval "$(_ROB_FOLDERS_COMPLETE=${my_source} rob_folders)"

# if there is alreay an active environment, refrain from scraping the empty paths
# (they must already there)
if [ -z "$ROB_FOLDERS_ACTIVE_ENV" ]; then
  export ROB_FOLDERS_EMPTY_CMAKE_PATH=${CMAKE_PREFIX_PATH}
  export ROB_FOLDERS_EMPTY_PATH=${PATH}
  export ROB_FOLDERS_EMPTY_LD_LIBRARY_PATH=${LD_LIBRARY_PATH}
  export ROB_FOLDERS_EMPTY_QML_IMPORT_PATH=${QML_IMPORT_PATH}
  export ROB_FOLDERS_EMPTY_PYTHONPATH=${PYTHONPATH}
  export ROB_FOLDERS_EMPTY_AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}
  export ROB_FOLDERS_EMPTY_COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH}
  export ROB_FOLDERS_EMPTY_PS1=${PS1}

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
else
  echo "Robot folder environment active: ($ROB_FOLDERS_ACTIVE_ENV)"
fi



# define some aliases for robot_folders
RED='\033[0;31m'
NC='\033[0m' # No Color
alias ce="fzirob change_environment"
alias cdr="fzirob cd"
alias cdros="fzirob cd ros"
alias cdcol="fzirob cd colcon"
alias cdmisc="fzirob cd misc"
alias cdhome="fzirob cd"
alias fm="fzirob make"
alias fmc="fzirob make colcon"
alias fmr="fzirob make ros"
alias fmcp="fzirob make colcon --colcon-args --packages-select"

makeros()
{
  echo -e "${RED}\`makeros\` is deprecated. Please use \`fmr\` instead.${NC}"
  fzirob make ros
}
makecol()
{
  echo -e "${RED}\`makecol\` is deprecated. Please use \`fmc\` instead.${NC}"
  fzirob make colcon
}

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
}


check_env()
{
  # for relevant env_vars
  #   for each entry
  #     if ROB_FOLDERS_CHECKOUT_DIR in entry and ROB_FOLDERS_ACTIVE_END not in entry
  #       cry
  VAR=$(echo "${CMAKE_PREFIX_PATH}" | sed 's/:/ /g')
  checkout_dir=$(rob_folders get_checkout_base_dir)
  for entry in $(echo "${VAR}"); do
    if [[ "${entry}" =~ ${checkout_dir} ]]; then
      if [[ ! "${entry}" =~ ${checkout_dir}/${ROB_FOLDERS_ACTIVE_ENV} ]]; then
        echo "\033[0;33mWARNING: CMAKE_PREFIX_PATH contains a path outside the current env: ${entry}\033[0m"
        echo "You probably built your catkin_ws the first time, when another workspace was sourced. To solve this, go to your catkin_ws, delete build and devel, open a new shell, source your environment, build your catkin_ws."
      fi
    fi
  done
}

reset_environment()
{
  export CMAKE_PREFIX_PATH=${ROB_FOLDERS_EMPTY_CMAKE_PATH}
  export PATH=${ROB_FOLDERS_EMPTY_PATH}
  export LD_LIBRARY_PATH=${ROB_FOLDERS_EMPTY_LD_LIBRARY_PATH}
  export QML_IMPORT_PATH=${ROB_FOLDERS_EMPTY_QML_IMPORT_PATH}
  export PYTHONPATH=${ROB_FOLDERS_EMPTY_PYTHONPATH}
  export AMENT_PREFIX_PATH=${ROB_FOLDERS_EMPTY_AMENT_PREFIX_PATH}
  export COLCON_PREFIX_PATH=${ROB_FOLDERS_EMPTY_COLCON_PREFIX_PATH}
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

      echo "rob_folders $@"

      rob_folders $@

      if [ $? -eq 0 ]; then
        if [ $1 = "change_environment" ] && [ "$2" != "--help"  ]; then
          reset_environment
          checkout_dir=$(rob_folders get_checkout_base_dir)

          if [ -f ${checkout_dir}/.cur_env ]; then
            # Since the python command writes the .cur_env file there is a race condition when
            # running change_environment commands in parallel. Thus it can happen that reading the
            # file returns an empty value. This race condition only occurs on a very high io load
            # and usually this while look should only be entered once.
            ROB_FOLDERS_ACTIVE_ENV=$(cat "${checkout_dir}"/.cur_env) || true
            while [ -z "$ROB_FOLDERS_ACTIVE_ENV" ]; do
              ROB_FOLDERS_ACTIVE_ENV=$(cat "${checkout_dir}"/.cur_env) || true
            done
            export ROB_FOLDERS_ACTIVE_ENV
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
            #check_env
            # Write current env to prompt
            if [ -z "${ROB_FOLDERS_DISABLE_PROMPT_MODIFICATION:-}" ] ; then
                env_prompt="[${ROB_FOLDERS_ACTIVE_ENV}]"
                if [ -n "${PS1##*"$env_prompt"*}" ]; then
                  PS1="${env_prompt} ${ROB_FOLDERS_EMPTY_PS1:-}"
                  export PS1
                fi
            fi
          else
            echo "Could not change environment"
          fi
        elif [ $1 = "add_environment" ] && [ "$2" != "--help"  ]; then
          if [ -z "$ROB_FOLDERS_ACTIVE_ENV" ]; then
            fzirob change_environment
            echo "Sourced new environment"
          else
            echo "Currently, there is a sourced environment ($ROB_FOLDERS_ACTIVE_ENV). Not sourcing new one."
          fi
        fi
      else
        return $?
      fi
    fi
  else
    rob_folders --help
  fi
}

# sourcing alias
source ${ROB_FOLDERS_BASE_DIR}/bin/rob_folders-complete.sh
