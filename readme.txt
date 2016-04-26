This directory contains a robot_folders workspace. robot_folders helps you
keeping track of different environment setups including multiple workspaces
such as ic_workspace, catkin_workspace and mca_workspace.

For initial setup, please call

  bin/install_robot_folders.sh

and follow the instructions on the screen. If you don't know whether you should
select 'bash' or 'zsh', you might want to select 'bash'

After installation open up a new terminal to use robot_folders. The main command
for using robot_folders is 'fzirob'. Type

  fzirob --help

to get an overview over all available commands.

If you used the old bash-based robot folders before, you might be happy to know
that many of the old aliases exist here, as well. To see a list of available 
aliases you can have a look at bin/fzirob_source.sh