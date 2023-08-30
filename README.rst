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
install pipx``).

.. code:: bash

   git clone https://ids-git.fzi.de/core/robot_folders
   pipx install ./robot_folders

Shell setup
-----------

In order to use ``robot_folders`` you'll have to call its source file. In case you have installed
``robot_folders`` using ``pipx`` as described above, do:

.. code:: bash

   echo "source ${HOME}/.local/pipx/venvs/robot-folders/bin/rob_folders_source.sh" >> ~/.bashrc

In case you installed it using the debian package, call

.. code:: bash

   echo "source /usr/bin/rob_folders_source.sh >> ~/.bashrc

Basic usage
-----------

After installation open up a new terminal to use robot_folders. The main
command for using robot_folders is ``fzirob``. Type

.. code:: bash

   fzirob --help

to get an overview over all available commands.

Customization
-------------

Configuration lies inside the **config** directory. The default
configuration is stored inside the file **userconfig_distribute.yaml**,
which will be copied to **userconfig.yaml** automatically. If you like
to change any configuration parameter, please change the
**userconfig.yaml** file.

It might happen, that your **userconfig.yaml** does not contain a
configuration parameter, yet. In this case, simply copy it from the
**userconfig_distribute.yaml** file.

The configuration is split into different sections which will be
explained in the following.

Build options
~~~~~~~~~~~~~

.. raw:: html

   <dl>

.. raw:: html

   <dt>

generator

.. raw:: html

   </dt>

.. raw:: html

   <dd>

Currently make and ninja can be used. If ninja is configured, but not
installed, building will throw an error.

.. raw:: html

   </dd>

.. raw:: html

   <dt>

cmake_flags

.. raw:: html

   </dt>

.. raw:: html

   <dd>

These flags will be passed to the cmake command.

.. raw:: html

   </dd>

.. raw:: html

   <dt>

make_threads

.. raw:: html

   </dt>

.. raw:: html

   <dd>

Number of threads that should be used with make. Only relevant when
generator is set to make.

.. raw:: html

   </dd>

.. raw:: html

   <dt>

install_catkin

.. raw:: html

   </dt>

.. raw:: html

   <dd>

If set to true, the build command will also install the catkin_workspace
(into the catkin_ws/install folder by default).

.. raw:: html

   </dd>

.. raw:: html

   <dt>

catkin_make_cmd

.. raw:: html

   </dt>

.. raw:: html

   <dd>

Set to catkin_make by default but can be changed to catkin build.

.. raw:: html

   </dd>

.. raw:: html

   </dl>

Directory options
~~~~~~~~~~~~~~~~~

.. raw:: html

   <dl>

.. raw:: html

   <dt>

checkout_dir

.. raw:: html

   </dt>

.. raw:: html

   <dd>

By default, environments are stored inside
${ROBOT_FOLDERS_BASE_DIR}/checkout If environments should be stored
somewhere else, specify this path here.

.. raw:: html

   </dd>

.. raw:: html

   <dt>

catkin_names

.. raw:: html

   </dt>

.. raw:: html

   <dd>

All first level subdirectories in an environment that match one of these
names will be treated as catkin workspaces. If you name yor catkin
workspaces differently, please specify this name here.

.. raw:: html

   </dd>

.. raw:: html

   </dl>

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
