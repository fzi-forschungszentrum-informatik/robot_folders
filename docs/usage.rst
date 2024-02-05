Usage
=====

The term "robot_folders" refers to a collection of utility scripts designed to
streamline and facilitate the management of workspaces within the IDS/TKS
framework. ``robot_folders`` is designed to enhance efficiency in handling
different environment setups, including multiple workspaces like a
``catkin_workspace`` and ``colcon_workspace``.

It focuses on optimizing the management of various subworkspaces, contributing
to a seamless development experience.

What is an environment?
-----------------------

An "environment" inside ``robot_folders`` consists of a set of workspaces. At
the moment the following types of workspaces ares supported:

1. **catkin_ws (ROS Workspace):**
   Modeled after the standard ROS (Robot Operating System) workspace,
   ``catkin_ws`` adheres to ROS tutorials and conventions, serving as a
   standardized environment for ROS-related development.

1. **colcon_ws (ROS 2 Workspace):**
   Modeled after the standard ROS 2 (Robot Operating System) workspace,
   ``colcon_ws`` adheres to ROS tutorials and conventions, serving as a
   standardized environment for ROS 2 related development.

1. **misc_ws:**
   The ``misc_ws`` adds the ability to add custom packages to your workspace.
   It simply adds its export paths to your environment. You'll have to take
   care to make sure the targets are built and exported there, yourself. See
   :ref:`misc_workspace:Misc Workspace` for details.


After installation, open up a new terminal to use ``robot_folders``. The main
command for using ``robot_folders`` is ``fzirob``. To get an overview of all available commands type

.. code:: bash

   fzirob --help

To utilize any command, adhere to the general syntax:

.. code:: bash

   fzirob command 'argument'


Creating an Environment
-----------------------
To create a new environment called ``ENV_NAME``, simply run 

.. code:: bash
  
   fzirob add_environment ENV_NAME

This will ask which types of workspaces should be generated for the new
environment. Depending on your specific setup further questions may be asked:

- When creating a catkin or colcon workspace with multiple installed ROS
  versions, you will be prompted for the ROS version being used for that
  workspace.
- When running on IDS/TKS workstations you will be asked whether the build and
  install folders should be placed outside of your ``checkout`` directory
  inside the ``$HOME/no_backup`` folder. This should be used on the
  workstations.


Upon completion, you will receive the confirmation: *"Environment setup for
'ENV_NAME` is complete."*


Sourcing an environment
-----------------------

To activate or source an environment, use the ``fzirob change_environment
ENV_NAME`` command. This command sources the appropriate setup scripts of the
environment depending on its contents. 

You can use tab completion on the environments so typing ``fzirob
change_environment`` and then pressing :kbd:`Tab` should list all environments
present.

Executing ``fzirob change_environment`` without any environment specified will
source the most recently sourced environment.


Compiling an environment
------------------------

To build an environment please use the ``fzirob make`` command. It has to be
invoked on an already sourced environment but can be called from anywhwere.

Without any further options this will build all the workspace present which it
knows how to build. So, if there is a catkin workspace present, it will build
that, if there's a colcon workspace this will be built.

You can also manually specify which workspace to build by using ``fzirob make
ros`` or ``fzirob make colcon``. When using ``fzirob make`` you don't have to
worry about the particular build command at all.

Default options for building environments such as the builder for catkin
workspaces or cmake arguments for a colcon workspace can be set in the
:ref:`configuration:Configuration`.

Cleaning an environment
-----------------------

Sometimes you want to completely rebuild your environment. ``fziron clean``
provides a command that will delete all build and installation artifacts from
your environment's workspaces. Before actual deletion it will show a list of
all the folders to be deleted with a safety prompt so you don't accidentally
delete things.

Navigating inside an environment
--------------------------------

You can use the ``fzirob cd`` command to navigate around in an environment. The
following examples show a couple of possible locations you can navigate to. For
the examples we have sourced the environment ``env_name`` and environments are
stored inside ``~/checkout``:

  .. code:: bash

    fzirob cd        # Env root folder e.g. ~/checkout/env_name
    fzirob cd ros    # Env catkin folder e.g. ~/checkout/env_name/catkin_ws
    fzirob cd colcon # Env colcon folder e.g. ~/checkout/env_name/colcon_ws

Again, tab completion will present the possible options for the currently sourced environment.

Start / Demo scripts
--------------------

For easy interaction with unknown environments ``robot_folders`` provides the
``fzirob run`` command. With that any executable file inside the environment's
``demos`` folder can be executed. So, if you've got an unknown environment and
just want to startup a predefined demo, simply source the environment, type
``fzirob run <tab>`` which should list all the possible demo scripts.

Sharing environments
--------------------

Often you'd like to share an environment with colleagues working on the same
project. While for colcon workspaces there exist external tools such as
vcstool2_, ``robot_folders`` provides extended functionality to that:

* Contents of multiple workspaces can be exported and imported at once (e.g.
  colcon workspace and misc workspace).
* Startup scripts are stored inside the exchange format for easy interaction.

Exporting an environment
~~~~~~~~~~~~~~~~~~~~~~~~

To export an environment, use the ``fzirob scrape_environment`` command. It scrapes an environment configuration into a config file, facilitating sharing with others. You'll have to provide the environment name and the target file as arguments e.g.

.. code:: bash

   fzirob scrape_environment env_name /tmp/env_name.yaml

In case you've got multiple remotes configured for repos inside your
environments, you will be queried which remote should be used for each
repository.

When providing ``--use_commit_id true``, the exact commit IDs get scraped
instead of branch names. which is rather useful if you want to save a "working
state" of your whole environment.

Creating a new environment with a configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you've got provided an environment configuration from somebody you can
create a new environment with that using

.. code:: bash
   
   fzirob add_environment --config-file /tmp/env_name.yaml other_env

which will create an environment called ``other_env`` with the configuration
from the previously exported ``env_name`` environment.

Adapting an environment with a configuration file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you already have an environment but you want to update it to match a config
file e.g. because some repositories have been added to the environment or some
branch names have been changed you can do this using the ``fzirob
adapt_environment`` command.

If there are repositories in your local environment that are not in the config
file you will be prompted whether you want to keep those repositories. If
branches or remotes in the config file differ from those present locally, you
will also be asked. You can override that to a default behavior using the
``--local_delete_policy`` and ``--local_override_policy`` options. 

Deleting an environment
-----------------------

If you want to delete an environment alltogether, you can use ``fzirob
delete_environment <env_name``. This will delete the conmplete environment
folder from your checkout directory.

.. _vcstool2: https://pypi.org/project/vcstool2/