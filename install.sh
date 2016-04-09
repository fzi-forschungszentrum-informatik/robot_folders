#!/bin/bash

# Get the base directory where the install script is located
ROB_FOLDERS_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

function manual_setup_instructions
{
    echo "Add ${ROB_FOLDERS_SCRIPT_DIR}/venv/bin to the BACK of your path."
    echo "    export PATH=\$PATH:${ROB_FOLDERS_SCRIPT_DIR}/venv/bin"
    echo
    echo "For bash completion (what you definitely want)."
    echo "    source ${ROB_FOLDERS_SCRIPT_DIR}/fzrob-complete.sh"
    echo
    echo "If you are a zsh user, you should have bashcompinit enabled in your .zshrc before sourcing the completion file:"
    echo "    autoload -U bashcompinit && bashcompinit"
}

shell_setup ()
{
    echo "# robot_folders setup" >> $1
    echo "export PATH=\$PATH:${ROB_FOLDERS_SCRIPT_DIR}/venv/bin" >> $1
    echo "source ${ROB_FOLDERS_SCRIPT_DIR}/fzrob-complete.sh" >> $1
}

function install
{
    echo "Creating virtual environment..."
    virtualenv venv
    . venv/bin/activate
    pip install Click

    echo "Installing robot_folders"
    pip install --editable .
}




install

mkdir -p ${ROB_FOLDERS_SCRIPT_DIR}/checkout

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
        echo "Added necessary parts to .zshrc. Have you loaded bashcompinit? Otherwise add"
        echo "    autoload -U bashcompinit && bashcompinit"
        echo "before the just inserted lines"
    else
        echo "Could not determine your shell type. Please perform manual setup:"
        manual_setup_instructions
    fi
else
    echo "You chose manual setup."
    manual_setup_instructions
fi


# deactivate the virtual env again
deactivate
