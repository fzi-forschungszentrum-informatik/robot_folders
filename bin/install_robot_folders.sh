#!/bin/bash

# Get the base directory where the install script is located
ROB_FOLDERS_BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"


SOURCE_CMD="source ${ROB_FOLDERS_BASE_DIR}/bin/fzirob_source.sh"

function manual_setup_instructions
{
    echo "Simply add ${SOURCE_CMD} in your .bashrc or zshrc respectively"
}

shell_setup ()
{
    # if the command hasn't been added already
    if ! grep -q "$SOURCE_CMD" $1;
    then
        echo "# robot_folders setup" >> $1
        echo $SOURCE_CMD >> $1
    fi
}

function install
{
    echo "Creating virtual environment..."
    pushd $ROB_FOLDERS_BASE_DIR/.robot_folders
    virtualenv venv
    . venv/bin/activate
    pip install Click
    pip install PyYaml

    echo "Installing robot_folders"
    pip install --editable .

    if [ -e "userconfig.py" ]
    then
        echo "userconfig.py already exists. Not overwriting."
        echo "If this is not your first install, but an update, you might want to check for config updates, though."
    else
        cp userconfig_distribute.py userconfig.py
    fi

    popd
}




install

mkdir -p ${ROB_FOLDERS_BASE_DIR}/checkout

# perform shell setup, if not done previously
if [ "$1" != "-q"  ]; then
  read -p "Do you want me to perform the .bashrc setup automatically? [Y/n] " -r do_setup

  if [[ "$do_setup" =~ ^[Yy]?$ ]]
  then
    read -p "Which shell are you running? Press enter for default bash: [bash/zsh] " -r shell_type
    # if running bash
    if [[ $shell_type == "bash" || $shell_type == "" ]]; then
      shell_setup "$HOME/.bashrc"
      echo "Added necessary parts to .bashrc"
    elif [[ $shell_type == zsh ]]; then
      shell_setup "$HOME/.zshrc"
      echo "Added necessary parts to .zshrc."
    else
      echo "Could not determine your shell type. Please perform manual setup:"
        manual_setup_instructions
    fi
  else
    echo "You chose manual setup."
    manual_setup_instructions
  fi
fi


# deactivate the virtual env again
deactivate
