Robot Folders
=============

Welcome to ``robot_folders``! ``robot_folders`` is a collection of utility scripts designed to
streamline and facilitate the management of workspaces used for (ROS) development. It is designed
to enhance efficiency in handling different environments, consisting of multiple workspaces like a
``catkin_workspace`` and ``colcon_workspace`` or a plain CMake workspace.

It focuses on optimizing the management of various subworkspaces, contributing
to a seamless development experience.


Quick start
------------

It is recommended to install robot_folders using ``pipx`` (You can install ``pipx`` using ``sudo apt
install pipx``). Please note: With pipx versions < 1.0 you'll have to provide the `--spec` flag

.. code:: bash

   # pipx >= 1.0 (from ubuntu 22.04 on)
   pipx install git+ssh://git@ids-git.fzi.de/core/robot_folders.git
   # pipx < 1.0 (ubuntu 20.04)
   pipx install --spec git+ssh://git@ids-git.fzi.de/core/robot_folders.git robot-folders


There is also a debian package available that is meant for centralized installations. Please contact
the package maintainers for further questions.

Upgrade
-------

To upgrade robot_folders using ``pipx`` do

.. code:: bash

   # pipx >= 1.0 (from ubuntu 22.04 on)
   pipx upgrade robot-folders
   # pipx < 1.0 (ubuntu 20.04)
   pipx upgrade --spec git+ssh://git@ids-git.fzi.de/core/robot_folders.git robot-folders

Shell setup
-----------

In order to use ``robot_folders`` you'll have to call its source file. In case you have installed
``robot_folders`` using ``pipx`` as described above, do:

.. code:: bash

   echo "source ${HOME}/.local/pipx/venvs/robot-folders/bin/rob_folders_source.sh" >> ~/.bashrc

In case you installed it using the debian package, call

.. code:: bash

   echo "source /usr/bin/rob_folders_source.sh" >> ~/.bashrc

Basic usage
-----------

After installation open up a new terminal to use robot_folders. The main
command for using robot_folders is ``fzirob``. Type

.. code:: bash

   fzirob --help

to get an overview over all available commands.
