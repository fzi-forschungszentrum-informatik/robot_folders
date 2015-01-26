Basic structure for creating a home directory on a robot. Includes handling different workspaces for different kinds of purposes/demos/projects.

* The git repository consists of three folders:

 bin       # Some useful scripts
 checkout  # The base dir for checking out different workspaces
 demos     # Folder for storing demo scripts


* On robot systems if you want to have bin checkout and demos in your home folder run in repos base folder:

 #move the git file to your home folder 
 cp -r .git ~        
 #go to your home folder
 cd ~
 #checkout the repository again into your home folder
 git checkout .
 #delete the robot_folder.
 rm -rf robot_folders

This will checkout this repository directly in your home folder. If there are no conflicts with existing folders this should work out of the box.

* Now you should add some useful aliases and environment variables to your ~/.bashrc:

 # Prepare robot_folders
 if [ -f ~/robot_folders/bin/prepare_robot_folders.bash ]; then
    . ~/robot_folders/bin/prepare_robot_folders.bash
 fi

or if you chose to have everything in your home folder:
 # Prepare robot_folders
 if [ -f ~/bin/prepare_robot_folders.bash ]; then
    . ~/bin/prepare_robot_folders.bash
 fi

* If you are finished you can open a new terminal and create your first project:
 add_fzi_project main

* At the end of the script you will get useful instructions that could be run afterwards. If you haven't worked with an ic_workspace or catkin_ws before you should first read the tutorials in the wiki:

http://idswiki.fzi.de/wiki/index.php/Dokumentation/Tutorials/IcWorkspace
http://idswiki.fzi.de/wiki/index.php/Dokumentation/Tutorials/Icl_ROS
