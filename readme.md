# Robot Folders
Welcome to robpt_folders! robot_folders helps you
keeping track of different environment setups including multiple workspaces
such as ic_workspace, catkin_workspace and mca_workspace.

You can source, create or build environments very easy.

## Installation
### Required packages
To use robot_folders you must have the following requirements installed:
 * python-pip
 * python-virtualenv
 * python-wstool

To install them on a ubuntu system just call
```bash
sudo apt-get install python-pip python-virtualenv python-wstool
```

### Initial setup
For initial setup, please call
```bash
bin/install_robot_folders.sh
```

and follow the instructions on the screen. If you don't know whether you should
select 'bash' or 'zsh', you might want to select 'bash'


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

  <dt>make_threads</dt>
  <dd>Number of threads that should be used with make. Only relevant when <b>generator</b> is set to <em>make</em>.</dd>

  <dt>install_ic</dt>
  <dd>If set to true, the build command will also install the ic_workspace (into the export folder by default).</dd>

  <dt>install_catkin</dt>
  <dd>If set to true, the build command will also install the catkin_workspace (into the catkin_ws/install folder by default).</dd>
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

## Transitioning from old robot_folders
If you used the old bash-based robot folders before, you might be happy to know
that many of the old aliases exist here, as well. To see a list of available
aliases you can have a look at bin/fzirob_source.sh

If you want to source additional files or want to run other various commands when
sourcing an environment, you can add the necessary commands to the
source_local.sh in that enviroment's folder.

NOTE: Unlike the old bash-based robot folders, LC_ALL is not set to C per default
anymore. If you want to keep this behaviour, use source_local.sh to set LC_ALL=C.

By default make will be used to build your workspaces. You can change your
default build system and other settings in .robot_folders/userconfig.py

