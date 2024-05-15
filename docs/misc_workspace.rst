Misc Workspace
==============

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

Exported (and sourced) paths
----------------------------

Currently, the following paths are added to the respective environment variables when sourcing an environment:

- ``misc_ws/export/lib`` gets added to ``$LD_LIBRARY_PATH``
- ``misc_ws/export/bin`` gets added to ``$PATH``
- ``misc_ws/export`` gets added to ``$CMAKE_PREFIX_PATH``

Misc workspace example
----------------------

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
