[![pipeline status](https://ids-git.fzi.de/core/robot_folders/badges/master/pipeline.svg)](https://ids-git.fzi.de/core/robot_folders/-/commits/master)
[![coverage report](https://ids-git.fzi.de/core/robot_folders/badges/master/coverage.svg)](https://ids-git.fzi.de/core/robot_folders/-/commits/master)

# Robot Folders
Welcome to robot_folders! robot_folders helps you
keeping track of different environment setups including multiple workspaces
such as ic_workspace, catkin_workspace and mca_workspace.

You can source, create or build environments very easy.

## Installation
### Required packages
To use robot_folders you must have the following requirements installed:
 * python3-pip

To install them on a ubuntu system just call
```bash
sudo apt-get install python3-pip
```

### Initial setup
For initial setup, please call
```bash
bin/install_robot_folders.sh
```

and follow the instructions on the screen. If you don't know whether you should
select 'bash' or 'zsh', you might want to select 'bash'

### Updating
To update robot_folders, simply pull from the repository and execute the installation script
afterwards.

```bash
cd <robot_folders_dir>
git pull
bin/install_robot_folders.sh
```

## Basic usage
After installation open up a new terminal to use robot_folders. The main command
for using robot_folders is 'fzirob'. Type
```bash
fzirob --help
```

to get an overview over all available commands.



## Customization
Configuration lies inside the **config** directory. The default configuration is stored inside the
file ***userconfig_distribute.yaml***, which will be copied to ***userconfig.yaml*** automatically. If you like
to change any configuration parameter, please change the ***userconfig.yaml*** file.

It might happen, that your ***userconfig.yaml*** does not contain a configuration parameter, yet.
In this case, simply copy it from the ***userconfig_distribute.yaml*** file.

The configuration is split into different sections which will be explained in the following.

### Build options
<dl>
  <dt>generator</dt>
  <dd>Currently <em>make</em> and <em>ninja</em> can be used. If <em>ninja</em> is configured, but not installed, building will throw an error.</dd>

  <dt>cmake_flags</dt>
  <dd>These flags will be passed to the cmake command.</dd>

  <dt>make_threads</dt>
  <dd>Number of threads that should be used with make. Only relevant when <b>generator</b> is set to <em>make</em>.</dd>

  <dt>install_ic</dt>
  <dd>If set to true, the build command will also install the ic_workspace (into the export folder by default).</dd>

  <dt>install_catkin</dt>
  <dd>If set to true, the build command will also install the catkin_workspace (into the catkin_ws/install folder by default).</dd>

  <dt>catkin_make_cmd</dt>
  <dd>Set to <em>catkin_make</em> by default but can be changed to <em>catkin build</em>.</dd>
</dl>

### Directory options
<dl>
  <dt>checkout_dir</dt>
  <dd>
    By default, environments are stored inside <b><em>${ROBOT_FOLDERS_BASE_DIR}/checkout</em></b>
    If environments should be stored somewhere else, specify this path here.
  </dd>

  <dt>catkin_names</dt>
  <dd>All first level subdirectories in an environment that match one of these names will be treated as catkin workspaces. If you name yor catkin workspaces differently, please specify this name here.</dd>
</dl>



### Misc workspace
**Note:** the misc workspace should be used with caution as it is an unconvenient way to build your software.

The misc workspace can be used to build plain cmake, fla or other types of git repositories, but the build procedure has to be managed manually by the user. The misc workspace has the following structure:

``` console
|-- misc_ws
  |-- export
  |-- repo-A
  |-- repo-B
  |-- ...
```

The misc workspace is included when the command
``` console
fzirob scrape_environment <workspace> <config-file>
```

is used and also applied when 
``` console
fzirob adapt_environment <workspace> <config-file>
```

or
 
``` console
# If your environment contains a misc_ws you probably want to built its contents first
# (see next section) before building any workspace depending on that. That's why the
# '--no_build' flag is activated in this example
fzirob add_environment <workspace> --config_file <config-file> --no_build
```

is used to share or save a workspace with others.

When sourcing an environment, the misc_ws export folder will be sourced ontop of the catkin_workspace and ic_workspace. This way, it will be available to other workspaces automatically.

### Misc workspace example

Assume that repository "repo-A" has build dependencies on repository "repo-B":
repo-B depends on repo-A. Then you can build the workspace manually by calling:

```console
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
```

### placement of `fzirob_source.sh`
If you alter your `~.bashrc` file after you've unstalled `robot_folders`, make sure that the line

```
source ${HOME}/robot_folders/bin/fzirob_source.sh
```

is always the **last** line when setting up your environment. `robot_folders` caches a couple of environment variables to make transitioning between different environments easier. Therefore, most changes to the environment that are made after that line will get overwritten once an environment is sourced using `robot_folders`. See the cache variables with the names `$ROB_FOLDERS_EMPTY_*` for details on which variables are cached.

## Transitioning from old robot_folders
If you used the old bash-based robot folders before, you might be happy to know
that many of the old aliases exist here, as well. To see a list of available
aliases you can have a look at bin/fzirob_source.sh

If you want to source additional files or want to run other various commands when
sourcing an environment, you can add the necessary commands to the
setup_local.sh in that enviroment's folder.

NOTE: Unlike the old bash-based robot folders, LC_ALL is not set to C per default
anymore. If you want to keep this behaviour, use source_local.sh to set LC_ALL=C.

By default make will be used to build your workspaces. You can change your
default build system and other settings in .robot_folders/userconfig.py

ATTENTION: The files setup.bash, setup.sh, setup.zsh from the old bash-based robot folders may 
shadow the new change_environment function of the python based robot folders. This can cause various problems, 
since the folder ~/.cmake/packages does not get removed on every change_environment command and the builds of 
two environments may be messed up and corrupted. When you switch to the new robot folders, please check for setup.* files 
and delete them if want to rely on the standard python robot folders.


