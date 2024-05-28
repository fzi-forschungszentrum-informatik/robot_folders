#
# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
"""implements the add functionality"""
import os
import stat
import subprocess
import sys

import click
import inquirer

import robot_folders.helpers.directory_helpers as dir_helpers
import robot_folders.helpers.build_helpers as build
import robot_folders.helpers.environment_helpers as environment_helpers
from robot_folders.helpers.ConfigParser import ConfigFileParser
from robot_folders.helpers.exceptions import ModuleException
from robot_folders.helpers.ros_version_helpers import *
from robot_folders.helpers.underlays import UnderlayManager


class EnvCreator(object):
    """Worker class that actually handles the environment creation"""

    def __init__(self, name, no_submodules=False):
        self.env_name = name
        self.no_submodules = no_submodules
        self.build_base_dir = dir_helpers.get_checkout_dir()
        self.demos_dir = os.path.join(
            dir_helpers.get_checkout_dir(), self.env_name, "demos"
        )

        self.misc_ws_directory = os.path.join(
            dir_helpers.get_checkout_dir(), self.env_name, "misc_ws"
        )
        self.misc_ws_build_directory = "to_be_set"
        self.misc_ws_rosinstall = None

        self.catkin_directory = os.path.join(
            dir_helpers.get_checkout_dir(), self.env_name, "catkin_ws"
        )
        self.catkin_build_directory = "to_be_set"
        self.cama_flags = ""
        self.catkin_rosinstall = ""

        self.colcon_directory = os.path.join(
            dir_helpers.get_checkout_dir(), self.env_name, "colcon_ws"
        )
        self.colcon_build_directory = "to_be_set"
        self.colcon_rosinstall = ""

        self.script_list = list()
        self.build = True

        self.create_catkin = False
        self.create_colcon = False
        self.create_misc_ws = False

        self.underlays = UnderlayManager(self.env_name)

    def create_new_environment(
        self,
        config_file,
        no_build,
        create_misc_ws,
        create_catkin,
        create_colcon,
        copy_cmake_lists,
        local_build,
        ros_distro,
        ros2_distro,
        underlays,
    ):
        """Worker method that does the actual job"""
        if os.path.exists(os.path.join(dir_helpers.get_checkout_dir(), self.env_name)):
            # click.echo("An environment with the name \"{}\" already exists. Exiting now."
            # .format(self.env_name))
            raise ModuleException(
                'Environment "{}" already exists'.format(self.env_name), "add"
            )

        has_nobackup = dir_helpers.check_build_on_nobackup(local_build)
        self.build_base_dir = dir_helpers.get_build_base_dir(has_nobackup)

        self.misc_ws_build_directory = os.path.join(
            self.build_base_dir, self.env_name, "misc_ws"
        )
        self.catkin_build_directory = os.path.join(
            self.build_base_dir, self.env_name, "catkin_ws", "build"
        )
        self.colcon_build_directory = os.path.join(
            self.build_base_dir, self.env_name, "colcon_ws", "build"
        )

        # If config file is give, parse it
        if config_file:
            self.parse_config(config_file)
        # Otherwise ask the user or check given flags
        else:
            if create_misc_ws == "ask":
                self.create_misc_ws = click.confirm(
                    "Would you like to create a misc workspace?", default=True
                )
            else:
                self.create_misc_ws = dir_helpers.yes_no_to_bool(create_misc_ws)

            if create_catkin == "ask":
                self.create_catkin = click.confirm(
                    "Would you like to create a catkin_ws?", default=True
                )
            else:
                self.create_catkin = dir_helpers.yes_no_to_bool(create_catkin)

            if create_colcon == "ask":
                self.create_colcon = click.confirm(
                    "Would you like to create a colcon_ws?", default=True
                )
            else:
                self.create_colcon = dir_helpers.yes_no_to_bool(create_colcon)

        click.echo('Creating environment with name "{}"'.format(self.env_name))

        catkin_creator = None
        if self.create_catkin:
            catkin_creator = environment_helpers.CatkinCreator(
                catkin_directory=self.catkin_directory,
                build_directory=self.catkin_build_directory,
                rosinstall=self.catkin_rosinstall,
                copy_cmake_lists=copy_cmake_lists,
                ros_distro=ros_distro,
                no_submodules=self.no_submodules,
            )
        colcon_creator = None
        if self.create_colcon:
            colcon_creator = environment_helpers.ColconCreator(
                colcon_directory=self.colcon_directory,
                build_directory=self.colcon_build_directory,
                rosinstall=self.colcon_rosinstall,
                ros2_distro=ros2_distro,
                no_submodules=self.no_submodules,
            )

        if underlays == "ask":
            self.underlays.query_underlays()
        elif underlays == "skip":
            pass
        else:
            raise NotImplementedError(
                "Manually passing underlays isn't implemented yet."
            )

        if self.underlays.underlays:
            if not no_build and (self.catkin_rosinstall or self.colcon_rosinstall):
                click.secho(
                    "Underlays selected without the 'no-build' option with a specified workspace. "
                    "Initial build will be deactivated. "
                    "Please manually build your environment by calling `fzirob make`.",
                    fg="yellow",
                )
            no_build = True

        # Let's get down to business
        self.create_directories()
        self.create_demo_docs()
        self.create_demo_scripts()

        if self.create_misc_ws:
            click.echo("Creating misc workspace")
            environment_helpers.MiscCreator(
                misc_ws_directory=self.misc_ws_directory,
                rosinstall=self.misc_ws_rosinstall,
                build_root=self.misc_ws_build_directory,
                no_submodules=self.no_submodules,
            )
        else:
            click.echo("Requested to not create a misc workspace")

        # Check if we should create a catkin workspace and create one if desired
        if catkin_creator:
            click.echo("Creating catkin_ws")
            catkin_creator.create()
        else:
            click.echo("Requested to not create a catkin_ws")

        if colcon_creator:
            click.echo("Creating colcon_ws")
            colcon_creator.create()
        else:
            click.echo("Requested to not create a colcon_ws")

        if not no_build:
            if self.create_catkin and self.catkin_rosinstall != "":
                ros_builder = build.CatkinBuilder(
                    name=catkin_creator.ros_distro, add_help_option=False
                )
                ros_builder.invoke(None)
            if self.create_colcon and self.colcon_rosinstall != "":
                ros2_builder = build.ColconBuilder(
                    name=colcon_creator.ros2_distro, add_help_option=False
                )
                ros2_builder.invoke(None)

    def create_directories(self):
        """Creates the directory skeleton with build_directories and symlinks"""
        os.mkdir(os.path.join(dir_helpers.get_checkout_dir(), self.env_name))
        os.mkdir(self.demos_dir)

        # Add a custom source file to the environment. Custom source commands go in here.
        env_source_file = open(
            os.path.join(
                dir_helpers.get_checkout_dir(), self.env_name, "setup_local.sh"
            ),
            "w",
        )
        env_source_file.write(
            "#This file is for custom source commands in this environment.\n"
        )
        env_source_file.write(
            '# zsh\nif [ -n "${ZSH_VERSION+1}" ]; then\n  # Get the base directory where the install script is located\n'
            '  setup_local_dir="$( cd "$( dirname "${(%):-%N}" )" && pwd )"\nfi\n\n'
            '# bash\nif [ -n "${BASH_VERSION+1}" ]; then\n'
            "  # Get the base directory where the install script is located\n"
            '  setup_local_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"\nfi\n'
        )
        env_source_file.close()

        self.underlays.write_underlay_file()

        os.symlink(
            os.path.join(dir_helpers.get_base_dir(), "bin", "source_environment.sh"),
            os.path.join(dir_helpers.get_checkout_dir(), self.env_name, "setup.sh"),
        )

    def parse_config(self, config_file):
        """Parses a config file to get an environment information"""
        parser = ConfigFileParser(config_file)

        (self.create_misc_ws, self.misc_ws_rosinstall) = parser.parse_misc_ws_config()

        # Parse catkin_workspace packages
        self.create_catkin, self.catkin_rosinstall = parser.parse_ros_config()

        # Parse colcon_workspace packages
        self.create_colcon, self.colcon_rosinstall = parser.parse_ros2_config()

        # Parse demo scripts and copy them to individual files
        self.script_list = parser.parse_demo_scripts()

    def create_demo_scripts(self):
        """If there are demo scripts given to the environment, create them."""
        click.echo("Found the following demo scripts:")
        for demo in self.script_list:
            click.echo(demo)
            filename = os.path.join(self.demos_dir, demo)
            with open(filename, mode="w") as script_file_content:
                script_file_content.write(self.script_list[demo])
                script_file_content.close()
            os.chmod(
                filename,
                stat.S_IRWXU
                | stat.S_IRGRP
                | stat.S_IXGRP
                | stat.S_IROTH
                | stat.S_IXOTH,
            )

    def create_demo_docs(self):
        """Creates a readme.txt in the demos folder to explain it's functionality"""
        doc_filename = os.path.join(self.demos_dir, "readme.txt")
        with open(doc_filename, "w") as out_file:
            docstring = """This folder can contain any executable files. These files can be run
    with the fzirob run command. When scraping an environment to a config file
    all demo scripts will be copied into the environment config, as well."""
            out_file.write(docstring)


@click.command("add_environment", short_help="Add a new environment")
@click.option("--config_file", help="Create an environment from a given config file.")
@click.option(
    "--no_build", is_flag=True, default=False, help="Do not perform an initial build."
)
@click.option(
    "--create_misc_ws",
    type=click.Choice(["yes", "no", "ask"]),
    default="ask",
    help="If set to 'yes', a folder for miscellaneous projects like \"fla\"-libraries or plain cmake projects is created and its export path is added to the environment.",
)
@click.option(
    "--create_catkin",
    type=click.Choice(["yes", "no", "ask"]),
    default="ask",
    help="If set to 'yes', a catkin-workspace is created without asking for it again.",
)
@click.option(
    "--create_colcon",
    type=click.Choice(["yes", "no", "ask"]),
    default="ask",
    help="If set to 'yes', a colcon-workspace is created without asking for it again.",
)
@click.option(
    "--copy_cmake_lists",
    type=click.Choice(["yes", "no", "ask"]),
    default="ask",
    help=(
        "If set to 'yes', the toplevel CMakeLists files of the catkin workspace is "
        "copied to the src folder of the catkin ws without asking for it again."
    ),
)
@click.option(
    "--local_build",
    type=click.Choice(["yes", "no", "ask"]),
    default="ask",
    help=(
        "If set to 'yes', the environment folder will be used for building directly. "
        " If set to 'no', builds will be done inside the `no_backup` directory"
    ),
)
@click.option(
    "--ros_distro",
    default="ask",
    help=(
        "If set, use this ROS1 distro instead of asking when multiple ROS1 distros are present on the system."
    ),
)
@click.option(
    "--ros2_distro",
    default="ask",
    help=(
        "If set, use this ROS2 distro instead of asking when multiple ROS2 distros are present on the system."
    ),
)
@click.option(
    "--no_submodules",
    default=False,
    is_flag=True,
    help="Prevent git submodules from being cloned",
)
@click.option(
    "--underlays",
    type=click.Choice(["ask", "skip"]),
    default="ask",
    help=(
        'When set to "ask" the user will be prompted for a list of underlays '
        'to be used. When set to "skip", no underlays will be configured.'
    ),
)
@click.argument("env_name", nargs=1)
def cli(
    env_name,
    config_file,
    no_build,
    create_misc_ws,
    create_catkin,
    create_colcon,
    copy_cmake_lists,
    local_build,
    ros_distro,
    ros2_distro,
    no_submodules,
    underlays,
):
    """Adds a new environment and creates the basic needed folders,
    e.g. a colcon_workspace and a catkin_ws."""
    environment_creator = EnvCreator(env_name, no_submodules=no_submodules)
    environment_creator.build = not no_build

    is_env_active = False
    if os.environ.get("ROB_FOLDERS_ACTIVE_ENV"):
        is_env_active = True
    os.environ["ROB_FOLDERS_ACTIVE_ENV"] = env_name

    try:
        environment_creator.create_new_environment(
            config_file,
            no_build,
            create_misc_ws,
            create_catkin,
            create_colcon,
            copy_cmake_lists,
            local_build,
            ros_distro,
            ros2_distro,
            underlays,
        )
    except subprocess.CalledProcessError as err:
        raise (ModuleException(str(err), "add"))
    except Exception as err:
        click.echo(err)
        click.echo("Something went wrong while creating the environment!")
        raise (ModuleException(str(err), "add"))
    click.echo("Initial workspace setup completed")

    if not is_env_active:
        click.echo("Writing env %s into .cur_env" % env_name)
        with open(
            os.path.join(dir_helpers.get_checkout_dir(), ".cur_env"), "w"
        ) as cur_env_file:
            cur_env_file.write("{}".format(env_name))
