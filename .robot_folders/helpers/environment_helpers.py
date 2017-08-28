import os
import subprocess

import click

import helpers.config_helpers as config_helpers
import helpers.build_helpers as build_helpers
import helpers.directory_helpers as dir_helpers

from yaml import dump as yaml_dump


class IcCreator(object):

    def __init__(self,
                 ic_directory,
                 build_directory,
                 packages=None,
                 package_versions=None,
                 grab_flags=None,
                 rosinstall=None):
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
            self.add_packages(packages, package_versions, grab_flags)
        else:
            self.add_rosinstall(rosinstall)
        self.create_build_folders()

    def add_packages(self, packages, package_versions, grab_flags):
        # If something goes wrong here, this will throw an exception, which is fine, as it shouldn't
        grab_command = ["./IcWorkspace.py", "grab", packages]
        if grab_flags is not None:
            grab_command.extend(grab_flags)
        process = subprocess.Popen(grab_command, cwd=self.ic_directory)
        process.wait()

        if package_versions is not None:
            for package in package_versions.keys():
                if package in packages:
                    click.echo("Checking out version {} of package {}".format(
                        package_versions[package], package))
                    package_dir = os.path.join(self.ic_directory, "packages", package)
                    click.echo("Package_dir: {}".format(package_dir))
                    process = subprocess.Popen(["git", "checkout", package_versions[package]],
                                               cwd=package_dir)
                    process.wait()
                else:
                    click.echo(
                        'Version for package {} given, however, '
                        'package is not listed in environment!'.format(package))

    def add_rosinstall(self, rosinstall):
        # if a rosinstall is specified, no base-grabbing is required
        # Dump the rosinstall to a file and use wstool for getting the packages
        rosinstall_filename = '/tmp/rob_folders_rosinstall'
        with open(rosinstall_filename, 'w') as f:
            yaml_dump(rosinstall, f)

        # If something goes wrong here, this will throw an exception, which is fine, as it shoudln't
        process = subprocess.Popen(["wstool", "init", "packages", rosinstall_filename],
                                   cwd=self.ic_directory)
        process.wait()
        os.remove(rosinstall_filename)

        # It is necessary to grab the base packages to get an icmaker
        grab_command = ["./IcWorkspace.py", "grab", "base"]
        process = subprocess.Popen(grab_command, cwd=self.ic_directory)
        process.wait()

    def create_build_folders(self):
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


class CatkinCreator(object):

    def __init__(self,
                 catkin_directory,
                 build_directory,
                 rosinstall,
                 copy_cmake_lists='ask'):
        self.catkin_directory = catkin_directory
        self.build_directory = build_directory
        self.copy_cmake_lists = copy_cmake_lists
        self.ros_distro=''

        self.ask_questions()
        ros_global_dir = "/opt/ros/{}".format(self.ros_distro)
        self.create_catkin_skeleton()
        self.build()
        self.clone_packages(rosinstall)

        if self.copy_cmake_lists:
            subprocess.check_call(["rm", "{}/src/CMakeLists.txt".format(self.catkin_directory)])
            subprocess.check_call(["cp",
                                   "{}/share/catkin/cmake/toplevel.cmake".format(ros_global_dir),
                                   "{}/src/CMakeLists.txt".format(self.catkin_directory)])

    def ask_questions(self):
        installed_ros_distros = os.listdir("/opt/ros")
        click.echo("Available ROS distributions: {}".format(installed_ros_distros))
        self.ros_distro = installed_ros_distros[0]
        if len(installed_ros_distros) > 1:
            self.ros_distro = click.prompt('Which ROS distribution would you like to use?',
                                           type=click.Choice(installed_ros_distros),
                                           default=installed_ros_distros[0])
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
        # We abuse the name parameter to code the ros distribution
        # if we're building for the first time.
        ros_builder = build_helpers.CatkinBuilder(name=self.ros_distro,
                                                  add_help_option=False)
        ros_builder.invoke(None)

    def create_catkin_skeleton(self):
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
        # copy packages
        if rosinstall != "":
            # Dump the rosinstall to a file and use wstool for getting the packages
            rosinstall_filename = '/tmp/rob_folders_rosinstall'
            with open(rosinstall_filename, 'w') as f:
                yaml_dump(rosinstall, f)

            process = subprocess.Popen(["wstool", "init", "src", rosinstall_filename],
                                       cwd=self.catkin_directory)
            process.wait()
            os.remove(rosinstall_filename)


class MCACreator(object):

    def __init__(self,
                 mca_directory,
                 build_directory,
                 mca_additional_repos):
        mca_repo_url = config_helpers.get_value_safe('repositories', 'mca_workspace_repo')

        subprocess.check_call(["git", "clone", mca_repo_url, mca_directory])

        self.mca_directory = mca_directory
        self.build_directory = build_directory

        self.add_from_config(mca_additional_repos)

        process = subprocess.Popen(["script/git_clone_base.py"],
                                   cwd=mca_directory)
        process.wait()

        process = subprocess.Popen(["script/ic/update_cmake.py"],
                                   cwd=mca_directory)
        process.wait()

        self.create_build_folders()

    def add_from_config(self, mca_additional_repos):
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
                    process = subprocess.Popen(["git", "checkout", package_version],
                                               cwd=libraries_dir)
                    process.wait()

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
                    process = subprocess.Popen(["git", "checkout", package_version],
                                               cwd=projects_dir)
                    process.wait()

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
                    process = subprocess.Popen(["git", "checkout", package_version],
                                               cwd=tools_dir)
                    process.wait()

    def create_build_folders(self):
        os.makedirs(self.build_directory)
        local_build_dir_name = os.path.join(self.mca_directory, "build")
        if local_build_dir_name != self.build_directory:
            os.symlink(self.build_directory, local_build_dir_name)
