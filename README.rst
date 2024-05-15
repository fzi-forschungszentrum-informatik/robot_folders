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
   pipx install git+https://github.com/fzi-forschungszentrum-informatik/robot_folders.git
   # pipx < 1.0 (ubuntu 20.04)
   pipx install --spec git+https://github.com/fzi-forschungszentrum-informatik/robot_folders.git robot-folders

Upgrade
-------

To upgrade robot_folders using ``pipx`` do

.. code:: bash

   # pipx >= 1.0 (from ubuntu 22.04 on)
   pipx upgrade robot-folders
   # pipx < 1.0 (ubuntu 20.04)
   pipx upgrade --spec git+https://github.com/fzi-forschungszentrum-informatik/robot_folders.git robot-folders

Shell setup
-----------

In order to use ``robot_folders`` you'll have to call its source file. How to do this depends on
the way you installed ``robot_folders`` and on the version of the installation tool.

In case you have installed
``robot_folders`` using ``pipx`` as described above (and given you use the bash shell), do:

.. code:: bash

   # pipx < 1.3.0 or if ${HOME}/.local/pipx already existed
   echo "source ${HOME}/.local/pipx/venvs/robot-folders/bin/rob_folders_source.sh" >> ~/.bashrc
   # pipx >= 1.3.0
   echo "source ${HOME}/.local/share/pipx/venvs/robot-folders/bin/rob_folders_source.sh" >> ~/.bashrc

In case you manually installed ``robot_folders`` using a python virtualenv the path is similarly

.. code:: bash

   echo "source <your-vurtualenv-path>/bin/rob_folders_source.sh" >> ~/.bashrc


Basic usage
-----------

After installation open up a new terminal to use robot_folders. The main
command for using robot_folders is ``fzirob``. Type

.. code:: bash

   fzirob --help

to get an overview over all available commands.
