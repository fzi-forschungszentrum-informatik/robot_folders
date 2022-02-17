"""
Module with helper classes to create workspaces
"""
import os
import subprocess

import click

import helpers.config_helpers as config_helpers
import helpers.build_helpers as build_helpers
import helpers.directory_helpers as dir_helpers
from helpers import config_helpers
from helpers.ros_version_helpers import *

from yaml import dump as yaml_dump


class IcCreator(object):
    """
    Class to create an IcWorkspace
    """

    def __init__(self,
                 ic_directory,
                 build_directory,
                 packages=None,
                 package_versions=None,
                 rosinstall=None,
                 grab_flags=None):
        self.ic_grab_flags = grab_flags
        base_url = config_helpers.get_value_safe_default("repositories", "ic_workspace_grab_base_url", "")
        # Check if there is a valid base url given for the i workspace.
        if base_url != "":
            self.ic_grab_flags = "--repo_uri_prefix=" + str(base_url)

        ic_repo_url = config_helpers.get_value_safe('repositories', 'ic_workspace_repo')
        ic_workspace_version = 'master'  # TODO: Parse this from somewhere

        if packages is None and rosinstall is None:
            click.echo('Either packages or rosinstall must be declared, none was given!')
            raise Exception
        if packages is not None and rosinstall is not None:
            click.echo('Either packages or rosinstall must be declared, not both!')
            raise Exception

        self.ic_directory = ic_directory
        self.build_directory = build_directory

        subprocess.check_call(["git",
                               "clone",
                               "-b",
                               ic_workspace_version,
                               ic_repo_url,
                               ic_directory])
        if packages is not None:
            self.add_packages(packages, package_versions)
        else:
            self.add_rosinstall(rosinstall)
        self.create_build_folders()

    def add_packages(self, packages, package_versions):
        """
        Adds the given packages with the IcWorkspace.py script. Package versions and grab_flags can
        be defined optionally.
        """
        # If something goes wrong here, this will throw an exception, which is fine, as it shouldn't
        grab_command = ["./IcWorkspace.py", "grab", packages]
        if self.ic_grab_flags is not None:
            grab_command += [ self.ic_grab_flags ]

        process = subprocess.check_call(grab_command, cwd=self.ic_directory)

        if package_versions is not None:
            for package in package_versions.keys():
                if package in packages:
                    click.echo("Checking out version {} of package {}".format(
                        package_versions[package], package))
                    package_dir = os.path.join(self.ic_directory, "packages", package)
                    click.echo("Package_dir: {}".format(package_dir))
                    process = subprocess.check_call(["git", "checkout", package_versions[package]],
                                                    cwd=package_dir)
                else:
                    click.echo(
                        'Version for package {} given, however, '
                        'package is not listed in environment!'.format(package))

    def add_rosinstall(self, rosinstall):
        """
        Adds packages from a rosinstall entry.
        """
        # if a rosinstall is specified, no base-grabbing is required
        # Dump the rosinstall to a file and use vcstool for getting the packages
        rosinstall_filename = '/tmp/rob_folders_rosinstall'
        with open(rosinstall_filename, 'w') as rosinstall_content:
            yaml_dump(rosinstall, rosinstall_content)

        os.makedirs(os.path.join(self.ic_directory, "packages"), exist_ok=True)

        # If something goes wrong here, this will throw an exception, which is fine, as it shoudln't
        process = subprocess.check_call(["vcs", "import", "--input", rosinstall_filename, "packages"],
                                        cwd=self.ic_directory)
        os.remove(rosinstall_filename)

        # It is necessary to grab the base packages to get an icmaker
        grab_command = ["./IcWorkspace.py", "grab", "base"]
        if self.ic_grab_flags is not None:
            grab_command += [ self.ic_grab_flags ]

        process = subprocess.check_call(grab_command, cwd=self.ic_directory)


    def create_build_folders(self):
        """
        Creates the necessary build directories in the file system. If a remote build is used (e.g.
        no_backup) then symlinks are created automatically.
        """
        (export_base_dir, _) = os.path.split(self.build_directory)
        export_directory = os.path.join(export_base_dir, "export")

        # Check if we need symlinks to the build and export directories and create them
        local_build_dir_name = os.path.join(self.ic_directory, "build")
        local_export_dir_name = os.path.join(self.ic_directory, "export")
        if local_build_dir_name != self.build_directory:
            os.symlink(self.build_directory, local_build_dir_name)
        if local_export_dir_name != export_directory:
            os.symlink(export_directory, local_export_dir_name)

        os.makedirs(self.build_directory)
        os.makedirs(export_directory)

class MiscCreator(object):
    """
    Class to create a misc workspace
    """

    def __init__(self,
                 misc_ws_directory,
                 build_root,
                 rosinstall=None):

        self.misc_ws_directory = misc_ws_directory
        self.build_root = build_root

        self.create_build_folders()
        self.add_rosinstall(rosinstall)

    def add_rosinstall(self, rosinstall):
        if rosinstall:
            # Dump the rosinstall to a file and use vcstool for getting the packages
            rosinstall_filename = '/tmp/rob_folders_rosinstall'
            with open(rosinstall_filename, 'w') as rosinstall_content:
                yaml_dump(rosinstall, rosinstall_content)

            os.makedirs(self.misc_ws_directory, exist_ok=True)
            process = subprocess.check_call(["vcs", "import", "--input", rosinstall_filename, "."],
                                            cwd=self.misc_ws_directory)
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

    def __init__(self,
                 catkin_directory,
                 build_directory,
                 rosinstall,
                 copy_cmake_lists='ask',
                 ros_distro='ask'):
        self.catkin_directory = catkin_directory
        self.build_directory = build_directory
        self.copy_cmake_lists = copy_cmake_lists
        self.ros_distro = ros_distro
        self.rosinstall = rosinstall

        self.ask_questions()
        self.ros_global_dir = "/opt/ros/{}".format(self.ros_distro)

    def create(self):
        self.create_catkin_skeleton()
        self.build()
        self.clone_packages(self.rosinstall)

        if self.copy_cmake_lists:
            subprocess.check_call(["rm", "{}/src/CMakeLists.txt".format(self.catkin_directory)])
            subprocess.check_call(["cp",
                                   "{}/share/catkin/cmake/toplevel.cmake".format(self.ros_global_dir),
                                   "{}/src/CMakeLists.txt".format(self.catkin_directory)])

    def ask_questions(self):
        """
        When creating a catkin workspace some questions need to be answered such as which ros
        version to use and whether to copy the CMakeLists.txt
        """
        if self.ros_distro == 'ask':
            installed_ros_distros = sorted(installed_ros_1_versions())
            click.echo("Available ROS distributions: {}".format(installed_ros_distros))
            self.ros_distro = installed_ros_distros[-1]
            if len(installed_ros_distros) > 1:
                self.ros_distro = click.prompt('Which ROS distribution would you like to use for'
                                               'catkin?',
                                               type=click.Choice(installed_ros_distros),
                                               default=installed_ros_distros[-1])
        click.echo("Using ROS distribution \'{}\'".format(self.ros_distro))
        if self.copy_cmake_lists == 'ask':
            self.copy_cmake_lists = click.confirm("Would you like to copy the top-level "
                                                  "CMakeLists.txt to the catkin"
                                                  " src directory instead of using a symlink?\n"
                                                  "(This is incredibly useful when using the "
                                                  "QtCreator.)",
                                                  default=True)
        else:
            self.copy_cmake_lists = dir_helpers.yes_no_to_bool(self.copy_cmake_lists)

    def build(self):
        """
        Launch the build process
        """
        # We abuse the name parameter to code the ros distribution
        # if we're building for the first time.
        ros_builder = build_helpers.CatkinBuilder(name=self.ros_distro,
                                                  add_help_option=False)
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
            rosinstall_filename = '/tmp/rob_folders_rosinstall'
            with open(rosinstall_filename, 'w') as rosinstall_content:
                yaml_dump(rosinstall, rosinstall_content)

            os.makedirs(os.path.join(self.catkin_directory, "src"), exist_ok=True)
            process = subprocess.check_call(["vcs", "import", "--input", rosinstall_filename, "src"],
                                            cwd=self.catkin_directory)
            os.remove(rosinstall_filename)

class ColconCreator(object):
    """
    Creates a colcon workspace
    """

    def __init__(self,
                 colcon_directory,
                 build_directory,
                 rosinstall,
                 ros2_distro='ask'):
        self.colcon_directory = colcon_directory
        self.build_directory = build_directory
        self.ros2_distro = ros2_distro
        self.rosinstall = rosinstall

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
        if self.ros2_distro == 'ask':
            installed_ros_distros = sorted(installed_ros_2_versions())
            click.echo("Available ROS2 distributions: {}".format(installed_ros_distros))
            self.ros2_distro = installed_ros_distros[-1]
            if len(installed_ros_distros) > 1:
                self.ros2_distro = click.prompt('Which ROS2 distribution would you like to use for '
                                                'colcon?',
                                               type=click.Choice(installed_ros_distros),
                                               default=installed_ros_distros[-1])
        click.echo("Using ROS2 distribution \'{}\'".format(self.ros2_distro))

    def build(self):
        """
        Launch the build process
        """
        # We abuse the name parameter to code the ros distribution
        # if we're building for the first time.
        ros2_builder = build_helpers.ColconBuilder(name=self.ros2_distro,
                                                  add_help_option=False)
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
            os.symlink(catkin_log_directory, local_log_dir_name)
            os.makedirs(colcon_install_directory)
            os.symlink(colcon_install_directory, local_install_dir_name)

    def clone_packages(self, rosinstall):
        """
        Clone packages from rosinstall structure
        """
        # copy packages
        if rosinstall != "":
            # Dump the rosinstall to a file and use vcstool for getting the packages
            rosinstall_filename = '/tmp/rob_folders_rosinstall'
            with open(rosinstall_filename, 'w') as rosinstall_content:
                yaml_dump(rosinstall, rosinstall_content)

            os.makedirs(os.path.join(self.colcon_directory, "src"), exist_ok=True)
            process = subprocess.check_call(
                ["vcs", "import", "--input", rosinstall_filename, "src"],
                cwd=self.colcon_directory
            )
            os.remove(rosinstall_filename)

class MCACreator(object):
    """
    Class to create an mca workspace
    """

    def __init__(self,
                 mca_directory,
                 build_directory,
                 mca_additional_repos):
        mca_repo_url = config_helpers.get_value_safe('repositories', 'mca_workspace_repo')

        subprocess.check_call(["git", "clone", mca_repo_url, mca_directory])

        self.mca_directory = mca_directory
        self.build_directory = build_directory

        self.add_from_config(mca_additional_repos)

        process = subprocess.check_call(["script/git_clone_base.py"], cwd=mca_directory)

        process = subprocess.check_call(["script/ic/update_cmake.py"], cwd=mca_directory)

        self.create_build_folders()

    def add_from_config(self, mca_additional_repos):
        """
        Parses a config entry and clones libraries, projects and tools
        """
        if 'libraries' in mca_additional_repos and mca_additional_repos['libraries'] is not None:
            for library in mca_additional_repos['libraries']:
                libraries_dir = os.path.join(self.mca_directory,
                                             'libraries',
                                             library['git']['local-name'])
                subprocess.check_call(["git", "clone", library['git']['uri'], libraries_dir])
                if 'version' in library['git']:
                    package_version = library['git']['version']
                    click.echo("Checking out version {} of package {}".format(
                        package_version, library['git']['local-name']))
                    process = subprocess.check_call(["git", "checkout", package_version],
                                                    cwd=libraries_dir)

        if 'projects' in mca_additional_repos and mca_additional_repos['projects'] is not None:
            for project in mca_additional_repos['projects']:
                click.echo("Project: {}".format(project))
                projects_dir = os.path.join(self.mca_directory,
                                            'projects',
                                            project['git']['local-name'])
                subprocess.check_call(["git", "clone", project['git']['uri'], projects_dir])
                if 'version' in project['git']:
                    package_version = project['git']['version']
                    click.echo("Checking out version {} of package {}".format(
                        package_version, project['git']['local-name']))
                    process = subprocess.check_call(["git", "checkout", package_version],
                                                    cwd=projects_dir)

        if 'tools' in mca_additional_repos and mca_additional_repos['tools'] is not None:
            for tool in mca_additional_repos['tools']:
                tools_dir = os.path.join(self.mca_directory,
                                         'tools',
                                         tool['git']['local-name'])
                subprocess.check_call(["git", "clone", tool['git']['uri'], tools_dir])
                if 'version' in tool['git']:
                    package_version = tool['git']['version']
                    click.echo("Checking out version {} of package {}".format(
                        package_version, tool['git']['local-name']))
                    process = subprocess.check_call(["git", "checkout", package_version],
                                                    cwd=tools_dir)

    def create_build_folders(self):
        """
        Creates necessary build folders for an mca workspace
        """
        os.makedirs(self.build_directory)
        local_build_dir_name = os.path.join(self.mca_directory, "build")
        if local_build_dir_name != self.build_directory:
            os.symlink(self.build_directory, local_build_dir_name)
