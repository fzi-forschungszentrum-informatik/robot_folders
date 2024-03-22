Configuration
=============

The default configuration is stored inside the file ``userconfig_distribute.yaml``,
which will be copied to ``~/.config/robot_folders.yaml`` automatically on first invocation.

.. literalinclude:: ../src/robot_folders/helpers/resources/userconfig_distribute.yaml
   :language: yaml

The configuration is split into different sections which will be
explained in the following.


Build options
-------------

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
-----------------

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

``no_backup_dir``
    Location where build and install steps should be performed if they should be outside of the
    checkout tree. If that folder exists, users will be prompted whether to build inside the
    ``no_backup_dir`` when creating a new environment.

