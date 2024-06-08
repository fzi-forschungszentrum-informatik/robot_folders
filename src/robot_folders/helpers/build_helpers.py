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
"""Module that helps building workspaces"""
import os
import re
import subprocess
import click

from robot_folders.helpers.directory_helpers import (
    get_active_env_path,
    mkdir_p,
    get_catkin_dir,
    get_colcon_dir,
)
from robot_folders.helpers.which import which
from robot_folders.helpers import compilation_db_helpers
from robot_folders.helpers import config_helpers
from robot_folders.helpers.exceptions import ModuleException
from robot_folders.helpers.option_helpers import SwallowAllOption


def get_cmake_flags():
    """Reads the configuration for default cmake flags"""
    generator = config_helpers.get_value_safe_default(
        section="build", value="generator", default="make"
    )

    cmake_flags = config_helpers.get_value_safe_default(
        section="build", value="cmake_flags", default=""
    )

    if generator == "ninja":
        cmake_flags = " ".join([cmake_flags, "-GNinja"])

    if which(generator) is None:
        click.echo(
            "WARNING: Generator '{}' was requested. However, "
            "that generator seems not to be installed. "
            "Will fallback to make instead.".format(generator)
        )
        cmake_flags = ""
    return cmake_flags


def get_cmake_flags():
    cmake_flags = config_helpers.get_value_safe_default(
        section="build", value="cmake_flags", default=""
    )
    return cmake_flags


class Builder(click.Command):
    """General builder class"""

    build_dir = "build"

    def get_build_command(self):
        """Determine whether to use make or ninja. If it cannot be guessed
        from the build directory, use the config value"""

        build_cmd = "make"

        cmake_cache_file = os.path.join(self.build_dir, "CMakeCache.txt")
        search_str = "CMAKE_MAKE_PROGRAM:FILEPATH="
        if os.path.isfile(cmake_cache_file):
            for line in open(cmake_cache_file):
                start = line.find(search_str)
                if start > -1:
                    # remove any trailing chars like newlines
                    build_cmd = os.path.basename(
                        os.path.normpath(line[start + len(search_str) :].rstrip())
                    )
        else:
            build_cmd = config_helpers.get_value_safe_default(
                section="build", value="generator", default="make"
            )

        if which(build_cmd) is None:
            click.echo(
                "WARNING: Generator '{}' was requested. However, "
                "that generator seems not to be installed. "
                "Will fallback to make instead.".format(build_cmd)
            )
            build_cmd = "make"

        if "make" in build_cmd:
            if "-j" not in build_cmd:
                num_threads = config_helpers.get_value_safe_default(
                    section="build", value="make_threads", default=2
                )
                build_cmd = " ".join([build_cmd, "-j", str(num_threads)])

        if self.should_install():
            build_cmd = " ".join([build_cmd, "install"])

        return build_cmd

    def check_previous_build(self, base_directory):
        """Checks whether the build directory exists and creates it if needed.
        Also performs an initial cmake command, if no CMakeCache.txt exists."""
        mkdir_p(self.build_dir)
        cmake_cache_file = os.path.join(self.build_dir, "CMakeCache.txt")
        if not os.path.isfile(cmake_cache_file):
            cmake_cmd = " ".join(["cmake", base_directory, get_cmake_flags()])
            click.echo("Starting initial build with command\n\t{}".format(cmake_cmd))
            try:
                process = subprocess.check_call(
                    ["bash", "-c", cmake_cmd], cwd=self.build_dir
                )
            except subprocess.CalledProcessError as err:
                raise (
                    ModuleException(err.message, "build_check_previous", err.returncode)
                )

    def should_install(self):
        """Checks if the build command should be run with the install option."""
        foobar = config_helpers.get_value_safe_default(
            section="build",
            value=self.get_install_key(),
            default=str(self.get_install_default()),
        )
        return foobar

    @classmethod
    def get_install_key(cls):
        """Returns the userconfig key for the install command of this builder."""
        return ""

    @classmethod
    def get_install_default(cls):
        """Returns the default install behavior for this builder,
        if none is provided by the user config"""
        return False


class ColconBuilder(Builder):
    """Builder class for colcon workspace"""

    def __init__(self, *args, **kwargs):
        params = [
            SwallowAllOption(
                ["--colcon-args"],
                nargs=-1,
                help="Arguments passed to colcon. Everything after this flag will be interpreted as"
                " colcon arguments. If this option is set, colcon arguments in the config are "
                "discarded",
                type=click.UNPROCESSED,
            )
        ]

        if "params" in kwargs and kwargs["params"]:
            kwargs["params"].extend(params)
        else:
            kwargs["params"] = params
        super().__init__(*args, **kwargs)

    def get_build_command(self, ros_distro, colcon_options):
        if colcon_options is None:
            colcon_options = config_helpers.get_value_safe_default(
                section="build", value="colcon_build_options", default=""
            )

        build_cmd = "colcon build"

        generator_flag = ""
        generator = config_helpers.get_value_safe_default(
            section="build", value="generator", default="make"
        )
        if generator == "ninja":
            generator_flag = "-GNinja"

        ros_global_dir = "/opt/ros/{}".format(ros_distro)

        if os.path.isdir(ros_global_dir):
            build_cmd_with_source = "source {}/setup.bash && {}".format(
                ros_global_dir, build_cmd
            )
            build_cmd = build_cmd_with_source

        final_cmd = " ".join(
            [
                build_cmd,
                colcon_options,
                "--cmake-args ",
                generator_flag,
                get_cmake_flags(),
            ]
        )

        click.echo("Building with command " + final_cmd)
        return final_cmd

    def invoke(self, ctx):
        colcon_dir = get_colcon_dir()
        click.echo("Building colcon_ws in {}".format(colcon_dir))

        if (
            ctx is not None
            and "colcon_args" in ctx.params
            and ctx.params["colcon_args"] is not None
        ):
            colcon_args = ctx.params["colcon_args"]
            colcon_args = " ".join(colcon_args)
        else:
            colcon_args = None

        # Colcon needs to build in an env that does not have the current workspace sourced
        # See https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#source-the-overlay
        my_env = os.environ.copy()
        keys_with_colcon_dir = [key for key, val in my_env.items() if colcon_dir in val]
        for key in keys_with_colcon_dir:
            my_env[key] = re.sub(colcon_dir + r"[^:]*", "", my_env[key])

        # We abuse the name to code the ros distribution if we're building for the first time.
        try:
            process = subprocess.check_call(
                ["bash", "-c", self.get_build_command(self.name, colcon_args)],
                cwd=colcon_dir,
                env=my_env,
            )
        except subprocess.CalledProcessError as err:
            raise (ModuleException(err.output, "build_colcon", err.returncode))


class CatkinBuilder(Builder):
    """Builder class for catkin workspace"""

    def get_build_command(self, catkin_dir, ros_distro):
        # default: make
        build_cmd = config_helpers.get_value_safe_default(
            section="build", value="catkin_make_cmd", default="catkin_make"
        )
        mkdir_p(self.build_dir)

        if build_cmd != "catkin_make_isolated":
            if os.path.isfile(
                os.path.join(
                    get_catkin_dir(), "build_isolated", "catkin_make_isolated.cache"
                )
            ):
                click.echo(
                    "WARNING: There seems to be an existing isolated build environment in "
                    "{}, however catkin_make_isolated is not configured as build command.\n"
                    "The workspace will be built using catkin_make_isolated. If this is not "
                    "the desired behavior, please remove the isolated build folder."
                )
                build_cmd = "catkin_make_isolated"

        generator_cmd = ""
        cmake_cache_file = os.path.join(catkin_dir, "build", "CMakeCache.txt")
        search_str = "CMAKE_MAKE_PROGRAM:FILEPATH="
        if os.path.isfile(cmake_cache_file):
            for line in open(cmake_cache_file):
                start = line.find(search_str)
                if start > -1:
                    # remove any trailing chars like newlines
                    if "ninja" in line:
                        if build_cmd == "catkin build":
                            raise (
                                ModuleException(
                                    "Catkin build does not support an option for using ninja. "
                                    "Please set the generator to make if you want to use catkin build",
                                    "build_ros",
                                    1,
                                )
                            )
                        else:
                            generator_cmd = "--use-ninja"
        else:
            generator = config_helpers.get_value_safe_default(
                section="build", value="generator", default="make"
            )
            if generator == "ninja":
                if build_cmd == "catkin build":
                    raise (
                        ModuleException(
                            "Catkin build does not support an option for using ninja. Please set the"
                            "generator to make if you want to use catkin build",
                            "build_ros",
                            1,
                        )
                    )
                else:
                    generator_cmd = "--use-ninja"

        install_cmd = ""
        if self.should_install():
            if build_cmd == "catkin_make":
                install_cmd = "install"
            elif build_cmd == "catkin_make_isolated":
                install_cmd = "--install"

        ros_global_dir = "/opt/ros/{}".format(ros_distro)

        if os.path.isdir(ros_global_dir):
            build_cmd_with_source = "source {}/setup.bash && {}".format(
                ros_global_dir, build_cmd
            )
            build_cmd = build_cmd_with_source

        final_cmd = " ".join([build_cmd, generator_cmd, install_cmd, get_cmake_flags()])
        click.echo("Building with command " + final_cmd)
        return final_cmd

    def invoke(self, ctx):
        catkin_dir = get_catkin_dir()
        self.build_dir = os.path.join(catkin_dir, "build")
        click.echo("Building catkin_workspace in {}".format(catkin_dir))

        # We abuse the name to code the ros distribution if we're building for the first time.
        try:
            process = subprocess.check_call(
                ["bash", "-c", self.get_build_command(catkin_dir, self.name)],
                cwd=catkin_dir,
            )
        except subprocess.CalledProcessError as err:
            raise (ModuleException(err.output, "build_ros", err.returncode))

        compilation_db_helpers.merge_compile_commands(
            self.build_dir, os.path.join(catkin_dir, "compile_commands.json")
        )

    def get_install_key(self):
        return "install_catkin"
