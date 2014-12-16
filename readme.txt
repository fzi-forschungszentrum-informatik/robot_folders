Basic structure for creating a home directory on a robot. Includes handling different workspaces for different kinds of purposes/demos/projects.

The git repository consists of three folders:

 bin       # Some useful scripts
 checkout  # The base dir for checking out different workspaces
 demos     # Folder for storing demo scripts


To start:
in repos base folder run:

 cp -r .git ~
 cd ~
 git checkout .

This will checkout this repository directly in your home folder. If there are no conflicts with existing folders this should work out of the box.

Now you should add some useful aliases and environment variables to your ~/.bashrc:

 # add ~/bin to PATH
 export PATH=$PATH:$HOME/bin

 # add an alias that sources the choose_environment.bash
 alias ce='source $HOME/bin/choose_environment.bash'

 # delete the .cmake folder since it is not working with mutliple workspaces
 (cd $HOME/.cmake/packages && rm -rf *)


If you are finished you can open a new terminal and create your first project:
 add_fzi_project main

At the end of the script you will get useful instructions that could be run afterwards. If you haven't worked with an ic_workspace or catkin_ws before you should first read the tutorials in the wiki:

http://idswiki.fzi.de/wiki/index.php/Dokumentation/Tutorials/IcWorkspace
http://idswiki.fzi.de/wiki/index.php/Dokumentation/Tutorials/Icl_ROS
