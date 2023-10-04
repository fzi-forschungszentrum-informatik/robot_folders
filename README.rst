|pipeline status| |coverage report|

Robot Folders
=============

Welcome to ``robot_folders``! ``robot_folders`` helps you keeping track of
different environment setups including multiple workspaces such as
catkin_workspace and colcon_workspace

You can source, create or build environments very easy.

Installation
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

Customization
-------------

The default configuration is stored inside the file ``userconfig_distribute.yaml``,
which will be copied to ``~/.config/robot_folders.yaml`` automatically on first invocation.

.. code:: yaml

   build: {
       generator: make,
       cmake_flags: "-DCMAKE_EXPORT_COMPILE_COMMANDS=1",
       make_threads: 4,
       install_catkin: False,
       catkin_make_cmd: catkin_make,
       colcon_build_options: "--symlink-install"
   }

   directories: {
       # if left blank, the default ~/checkout will be used
       checkout_dir: ,
       catkin_names: ["catkin_workspace", "catkin_ws"],
       colcon_names: ["colcon_workspace", "colcon_ws", "dev_ws"]
   }

The configuration is split into different sections which will be
explained in the following.

Build options
~~~~~~~~~~~~~

``generator``
    Currently make and ninja can be used. If ninja is configured, but not
    installed, building will throw an error.

``cmake_flags``
    These flags will be passed to the cmake command.

``make_threads``
    Number of threads that should be used with make. Only relevant when
    generator is set to make.

``install_catkin``
    If set to true, the build command will also install the catkin_workspace
    (into the catkin_ws/install folder by default).

``catkin_make_cmd``
    Set to catkin_make by default but can be changed to catkin build.

``colcon_build_options``
    Options passed to each ``colcon build`` invocation that is piped through ``fzirob make``.

Directory options
~~~~~~~~~~~~~~~~~

``checkout_dir``
    By default, environments are stored inside
    ~/checkout. If environments should be stored
    somewhere else, specify this path here. This **must** be an absolute path, but ``${HOME}/`` or
    ``~/`` can be used, as well.

``catkin_names``
    All first level subdirectories in an environment that match one of these
    names will be treated as catkin workspaces. If you name yor catkin
    workspaces differently, please specify this name here.

``colcon_names``
    All first level subdirectories in an environment that match one of these
    names will be treated as colcon workspaces. If you name yor colcon
    workspaces differently, please specify this name here.

Misc workspace
~~~~~~~~~~~~~~

**Note:** the misc workspace should be used with caution as it is an
unconvenient way to build your software.

The misc workspace can be used to build plain cmake, fla or other types
of git repositories, but the build procedure has to be managed manually
by the user. The misc workspace has the following structure:

.. code:: bash

   |-- misc_ws
     |-- export
     |-- repo-A
     |-- repo-B
     |-- ...

The misc workspace is included when the command

.. code:: bash

   fzirob scrape_environment <workspace> <config-file>

is used and also applied when

.. code:: bash

   fzirob adapt_environment <workspace> <config-file>

or

.. code:: bash

   # If your environment contains a misc_ws you probably want to built its contents first
   # (see next section) before building any workspace depending on that. That's why the
   # '--no_build' flag is activated in this example
   fzirob add_environment <workspace> --config_file <config-file> --no_build

is used to share or save a workspace with others.

When sourcing an environment, the misc_ws export folder will be sourced
ontop of the catkin_workspace / colcon workspace. This way, it will be
available to other workspaces automatically.

Misc workspace example
~~~~~~~~~~~~~~~~~~~~~~

Assume that repository “repo-A” has build dependencies on repository
“repo-B”: repo-B depends on repo-A. Then you can build the workspace
manually by calling:

.. code:: bash

   cd repo-B
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=../../export -DBUILD_SHARED_LIBS=1
   make
   make install
   cd ../../repo-A
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=../../export -DBUILD_SHARED_LIBS=1
   make
   make install

.. |pipeline status| image:: https://ids-git.fzi.de/core/robot_folders/badges/master/pipeline.svg
   :target: https://ids-git.fzi.de/core/robot_folders/-/commits/master
.. |coverage report| image:: https://ids-git.fzi.de/core/robot_folders/badges/master/coverage.svg
   :target: https://ids-git.fzi.de/core/robot_folders/-/commits/master
