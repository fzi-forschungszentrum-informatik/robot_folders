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
"""
Module with helper classes to create workspaces
"""
import os
import subprocess

import click

import inquirer

import robot_folders.helpers.config_helpers as config_helpers
import robot_folders.helpers.build_helpers as build_helpers
import robot_folders.helpers.directory_helpers as dir_helpers
from robot_folders.helpers import config_helpers
from robot_folders.helpers.ros_version_helpers import *

from yaml import dump as yaml_dump


class MiscCreator(object):
    """
    Class to create a misc workspace
    """

    def __init__(
        self, misc_ws_directory, build_root, rosinstall=None, no_submodules=False
    ):

        self.misc_ws_directory = misc_ws_directory
        self.build_root = build_root
        self.no_submodules = no_submodules

        self.create_build_folders()
        self.add_rosinstall(rosinstall)

    def add_rosinstall(self, rosinstall):
        if rosinstall:
            # Dump the rosinstall to a file and use vcstool for getting the packages
            rosinstall_filename = "/tmp/rob_folders_rosinstall"
            with open(rosinstall_filename, "w") as rosinstall_content:
                yaml_dump(rosinstall, rosinstall_content)

            os.makedirs(self.misc_ws_directory, exist_ok=True)
            if self.no_submodules:
                process = subprocess.check_call(
                    ["vcs", "import", "--input", rosinstall_filename, "."],
                    cwd=self.misc_ws_directory,
                )
            else:
                process = subprocess.check_call(
                    [
                        "vcs",
                        "import",
                        "--recursive",
                        "--input",
                        rosinstall_filename,
                        ".",
                    ],
                    cwd=self.misc_ws_directory,
                )

            os.remove(rosinstall_filename)

    def create_build_folders(self):
        """
        Creates the necessary export directory of the misc workspace in the file system. If a remote build is used (e.g.
        no_backup) then symlinks are created automatically.
        """
        export_directory = os.path.join(self.build_root, "export")
        local_export_dir_name = os.path.join(self.misc_ws_directory, "export")

        if local_export_dir_name != export_directory:
            os.symlink(export_directory, local_export_dir_name)

        os.makedirs(export_directory)


class CatkinCreator(object):
    """
    Creates a catkin workspace
    """

    def __init__(
        self,
        catkin_directory,
        build_directory,
        rosinstall,
        copy_cmake_lists="ask",
        ros_distro="ask",
        no_submodules=False,
    ):
        self.catkin_directory = catkin_directory
        self.build_directory = build_directory
        self.copy_cmake_lists = copy_cmake_lists
        self.ros_distro = ros_distro
        self.rosinstall = rosinstall
        self.no_submodules = no_submodules

        self.ask_questions()
        self.ros_global_dir = "/opt/ros/{}".format(self.ros_distro)

    def create(self):
        self.create_catkin_skeleton()
        self.build()
        self.clone_packages(self.rosinstall)

        if self.copy_cmake_lists:
            if os.path.exists(
                os.path.join(self.catkin_directory, "src", "CMakeLists.txt")
            ):
                subprocess.check_call(
                    ["rm", "{}/src/CMakeLists.txt".format(self.catkin_directory)]
                )
                subprocess.check_call(
                    [
                        "cp",
                        "{}/share/catkin/cmake/toplevel.cmake".format(
                            self.ros_global_dir
                        ),
                        "{}/src/CMakeLists.txt".format(self.catkin_directory),
                    ]
                )

    def ask_questions(self):
        """
        When creating a catkin workspace some questions need to be answered such as which ros
        version to use and whether to copy the CMakeLists.txt
        """
        if self.ros_distro == "ask":
            installed_ros_distros = installed_ros_1_versions()
            if installed_ros_distros:
                installed_ros_distros = sorted(installed_ros_distros)
                self.ros_distro = installed_ros_distros[-1]
                if len(installed_ros_distros) > 1:
                    questions = [
                        inquirer.List(
                            "ros_distro",
                            message="Which ROS distribution would you like to use for catkin?",
                            choices=installed_ros_distros,
                        ),
                    ]
                    self.ros_distro = inquirer.prompt(questions)["ros_distro"]
            else:
                self.ros_distro = None
        click.echo("Using ROS distribution '{}'".format(self.ros_distro))
        if self.copy_cmake_lists == "ask":
            self.copy_cmake_lists = click.confirm(
                "Would you like to copy the top-level "
                "CMakeLists.txt to the catkin"
                " src directory instead of using a symlink?\n"
                "(This is incredibly useful when using the "
                "QtCreator.)",
                default=True,
            )
        else:
            self.copy_cmake_lists = dir_helpers.yes_no_to_bool(self.copy_cmake_lists)

    def build(self):
        """
        Launch the build process
        """
        # We abuse the name parameter to code the ros distribution
        # if we're building for the first time.
        ros_builder = build_helpers.CatkinBuilder(
            name=self.ros_distro, add_help_option=False
        )
        ros_builder.invoke(None)

    def create_catkin_skeleton(self):
        """
        Creates the workspace skeleton and if necessary the relevant folders for remote build (e.g.
        no_backup)
        """
        # Create directories and symlinks, if necessary
        os.mkdir(self.catkin_directory)
        os.mkdir(os.path.join(self.catkin_directory, "src"))
        os.makedirs(self.build_directory)

        local_build_dir_name = os.path.join(self.catkin_directory, "build")
        (catkin_base_dir, _) = os.path.split(self.build_directory)

        catkin_devel_directory = os.path.join(catkin_base_dir, "devel")
        local_devel_dir_name = os.path.join(self.catkin_directory, "devel")
        click.echo("devel_dir: {}".format(catkin_devel_directory))

        catkin_install_directory = os.path.join(catkin_base_dir, "install")
        local_install_dir_name = os.path.join(self.catkin_directory, "install")
        click.echo("install_dir: {}".format(catkin_install_directory))

        if local_build_dir_name != self.build_directory:
            os.symlink(self.build_directory, local_build_dir_name)
            os.makedirs(catkin_devel_directory)
            os.symlink(catkin_devel_directory, local_devel_dir_name)
            os.makedirs(catkin_install_directory)
            os.symlink(catkin_install_directory, local_install_dir_name)

    def clone_packages(self, rosinstall):
        """
        Clone in packages froma rosinstall structure
        """
        # copy packages
        if rosinstall != "":
            # Dump the rosinstall to a file and use vcstools for getting the packages
            rosinstall_filename = "/tmp/rob_folders_rosinstall"
            with open(rosinstall_filename, "w") as rosinstall_content:
                yaml_dump(rosinstall, rosinstall_content)

            os.makedirs(os.path.join(self.catkin_directory, "src"), exist_ok=True)
            if self.no_submodules:
                process = subprocess.check_call(
                    ["vcs", "import", "--input", rosinstall_filename, "src"],
                    cwd=self.catkin_directory,
                )
            else:
                process = subprocess.check_call(
                    [
                        "vcs",
                        "import",
                        "--recursive",
                        "--input",
                        rosinstall_filename,
                        "src",
                    ],
                    cwd=self.catkin_directory,
                )

            os.remove(rosinstall_filename)


class ColconCreator(object):
    """
    Creates a colcon workspace
    """

    def __init__(
        self,
        colcon_directory,
        build_directory,
        rosinstall,
        ros2_distro="ask",
        no_submodules=False,
    ):
        self.colcon_directory = colcon_directory
        self.build_directory = build_directory
        self.ros2_distro = ros2_distro
        self.rosinstall = rosinstall
        self.no_submodules = no_submodules

        self.ask_questions()

    def create(self):
        """Actually creates the workspace"""
        self.create_colcon_skeleton()
        self.build()
        self.clone_packages(self.rosinstall)

    def ask_questions(self):
        """
        When creating a colcon workspace some questions need to be answered such as which ros
        version to use
        """
        if self.ros2_distro == "ask":
            installed_ros_distros = installed_ros_2_versions()
            if installed_ros_distros:
                installed_ros_distros = sorted(installed_ros_distros)
                self.ros2_distro = installed_ros_distros[-1]
                if len(installed_ros_distros) > 1:
                    questions = [
                        inquirer.List(
                            "ros_distro",
                            message="Which ROS2 distribution would you like to use for colcon?",
                            choices=installed_ros_distros,
                        ),
                    ]
                    self.ros2_distro = inquirer.prompt(questions)["ros_distro"]
            else:
                self.ros2_distro = None
        click.echo("Using ROS2 distribution '{}'".format(self.ros2_distro))

    def build(self):
        """
        Launch the build process
        """
        # We abuse the name parameter to code the ros distribution
        # if we're building for the first time.
        ros2_builder = build_helpers.ColconBuilder(
            name=self.ros2_distro, add_help_option=False
        )
        ros2_builder.invoke(None)

    def create_colcon_skeleton(self):
        """
        Creates the workspace skeleton and if necessary the relevant folders for remote build (e.g.
        no_backup)
        """
        # Create directories and symlinks, if necessary
        os.mkdir(self.colcon_directory)
        os.mkdir(os.path.join(self.colcon_directory, "src"))
        os.makedirs(self.build_directory)

        local_build_dir_name = os.path.join(self.colcon_directory, "build")
        (colcon_base_dir, _) = os.path.split(self.build_directory)

        colcon_log_directory = os.path.join(colcon_base_dir, "log")
        local_log_dir_name = os.path.join(self.colcon_directory, "log")
        click.echo("log_dir: {}".format(colcon_log_directory))

        colcon_install_directory = os.path.join(colcon_base_dir, "install")
        local_install_dir_name = os.path.join(self.colcon_directory, "install")
        click.echo("install_dir: {}".format(colcon_install_directory))

        if local_build_dir_name != self.build_directory:
            os.symlink(self.build_directory, local_build_dir_name)
            os.makedirs(colcon_log_directory)
            os.symlink(colcon_log_directory, local_log_dir_name)
            os.makedirs(colcon_install_directory)
            os.symlink(colcon_install_directory, local_install_dir_name)

    def clone_packages(self, rosinstall):
        """
        Clone packages from rosinstall structure
        """
        # copy packages
        if rosinstall != "":
            # Dump the rosinstall to a file and use vcstool for getting the packages
            rosinstall_filename = "/tmp/rob_folders_rosinstall"
            with open(rosinstall_filename, "w") as rosinstall_content:
                yaml_dump(rosinstall, rosinstall_content)

            os.makedirs(os.path.join(self.colcon_directory, "src"), exist_ok=True)
            if self.no_submodules:
                process = subprocess.check_call(
                    ["vcs", "import", "--input", rosinstall_filename, "src"],
                    cwd=self.colcon_directory,
                )
            else:
                process = subprocess.check_call(
                    [
                        "vcs",
                        "import",
                        "--recursive",
                        "--input",
                        rosinstall_filename,
                        "src",
                    ],
                    cwd=self.colcon_directory,
                )

            os.remove(rosinstall_filename)
