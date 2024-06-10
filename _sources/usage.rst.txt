Usage
=====

This page should give you an overview of different tasks that can be performed using
``robot_folders``. It is kind of an inverse documentation to the command documentation that comes
with the command line interface.

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
environment.

One of the questions will be which underlay you'd like to use. If you don't know what underlays are
and what to use them for, you can just leave the selection empty and press enter. If you want to
know more, see :ref:`usage:Using underlay environments`.

Depending on your specific setup further questions may be asked:

- When creating a catkin or colcon workspace with multiple installed ROS
  versions, you will be prompted for the ROS version being used for that
  workspace.
- If you use an automatic backup strategy for your machine you probably do not want to backup the
  build and install artifacts from your workspaces, but only the sources. For that purpose
  ``robot_folders`` allows building and installing outside of your checkout tree and creating
  symlinks at the respecitve places instead. Whenever the configured backup location exists on the
  system (By default that's ``$HOME/no_backup``, that can be changed in the
  :ref:`configuration:Configuration`.) you will be prompted whether
  you want to build inside the checkout tree or in the no-backup folder.

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

Using underlay environments
---------------------------

``robot_folders`` supports using underlay environments that will be sourced during sourcing the
current environment. With this it is possible to keep parts of your setup in another workspace.
Imaging for example, you have two packages: "Application" and "Library". "Application" is depending
on "Library", but there are also other packages in other environments depending on "Library". You
can reuse your local "Library" installation by putting it into its own environment and use that as
an underlay for your environment containing "Application" (and for all the other environments with
packages depending on "Library").

This way, you'll only have to keep "Library" up to date in one place.

Another use case is if you want to test your "Application" against different versions of "Library".
You could keep compiled versions of "Library" in separate environments and select the correct one
as an underlay for your application's environment. You'll only have to recompile your application
while with keeping "Library" inside your application environment you would have to go the to
library package, switch branches and rebuild the library and application package.

When creating new environment you will be prompted for the underlays to be used.

Underlays will be stored in a ``underlays.txt`` file inside your environment's folder. You can
either edit that file manually or use the ``fzirob manage_underlays`` command to change the
environment's underlays.

.. note::
   The underlay file will not get deleted by ``fzirob clean``. Although you will be asked for the
   ROS distribution to use (if more than one is installed), underlay configuration persists.

.. note::
   You can stack underlays. That means you can create dependency chains e.g. env1 -> env2 -> env3

.. note::
   Currently, initial build isn't supported when using underlay workspaces. When specifying an
   environment config when creating a new environment the user will have to manually trigger the
   build process after initially creating the environment. Usually, a simple `fzirob make` should
   do the trick.

.. warning::
   Currently there is no check for cyclic environment dependency. Please make sure you do not run
   into this problem.

.. warning::
   The order in which environments are sourced is depending on the order in which they are written
   into the ``underlays.txt`` file. That order might be alterd by the ``fzirob manage_underlays``
   command. If you depend on an order, you may instead consider stacking underlays depending on
   each other.

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

When building a colcon workspace, you can pass arguments to colcon using the ``--colcon-args``
option. For example to selectivly build a package called ``foobar`` and install it using symlinks
you can call ``fzirob make --colcon-args --packages-select foobar --symlink-install``.

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
