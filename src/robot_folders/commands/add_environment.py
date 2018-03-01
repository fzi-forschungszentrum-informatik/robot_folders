"""implements the add functionality"""
import os
import stat
import click

import helpers.directory_helpers as dir_helpers
import helpers.build_helpers as build
import helpers.environment_helpers as environment_helpers
from helpers.ConfigParser import ConfigFileParser


class EnvCreator(object):
    """Worker class that actually handles the environment creation"""

    def __init__(self, name):
        self.env_name = name
        self.build_base_dir = dir_helpers.get_checkout_dir()
        self.demos_dir = os.path.join(dir_helpers.get_checkout_dir(), self.env_name, 'demos')

        self.ic_directory = os.path.join(
            dir_helpers.get_checkout_dir(),
            self.env_name,
            "ic_workspace")
        self.ic_packages = "base"
        self.ic_package_versions = {}
        self.ic_grab_flags = []
        self.ic_cmake_flags = ""
        self.ic_rosinstall = None
        self.ic_build_directory = 'to_be_set'
        self.ic_flags = []

        self.catkin_directory = os.path.join(
            dir_helpers.get_checkout_dir(), self.env_name, "catkin_ws")
        self.catkin_build_directory = 'to_be_set'
        self.cama_flags = ""
        self.catkin_rosinstall = ""

        self.mca_directory = os.path.join(
            dir_helpers.get_checkout_dir(),
            self.env_name,
            "mca_workspace")
        self.mca_build_directory = 'to_be_set'
        self.mca_cmake_flags = ""
        self.mca_additional_repos = ""

        self.script_list = list()
        self.build = True

        self.create_ic = False
        self.create_catkin = False
        self.create_mca = False

    def create_new_environment(self,
                               config_file,
                               no_build,
                               create_ic,
                               create_catkin,
                               create_mca,
                               copy_cmake_lists,
                               local_build):
        """Worker method that does the actual job"""
        if os.path.exists(os.path.join(dir_helpers.get_checkout_dir(), self.env_name)):
            click.echo("An environment with the name \"{}\" already exists. Exiting now."
                       .format(self.env_name))
            raise Exception

        has_nobackup = dir_helpers.check_nobackup(local_build)
        self.build_base_dir = dir_helpers.get_build_base_dir(has_nobackup)

        # Build directories
        self.ic_build_directory = os.path.join(self.build_base_dir,
                                               self.env_name,
                                               "ic_workspace",
                                               "build")
        self.catkin_build_directory = os.path.join(self.build_base_dir,
                                                   self.env_name,
                                                   "catkin_ws",
                                                   "build")
        self.mca_build_directory = os.path.join(self.build_base_dir,
                                                self.env_name,
                                                "mca_workspace",
                                                "build")

        # If config file is give, parse it
        if config_file:
            self.parse_config(config_file)
        # Otherwise ask the user or check given flags
        else:
            if create_ic == 'ask':
                self.create_ic = click.confirm("Would you like to create an ic_workspace?",
                                               default=True)
            else:
                self.create_ic = dir_helpers.yes_no_to_bool(create_ic)

            if create_catkin == 'ask':
                self.create_catkin = click.confirm("Would you like to create a catkin_ws?",
                                                   default=True)
            else:
                self.create_catkin = dir_helpers.yes_no_to_bool(create_catkin)

            if create_mca == 'ask':
                self.create_mca = click.confirm("Would you like to create an mca_workspace?",
                                                default=True)
            else:
                self.create_mca = dir_helpers.yes_no_to_bool(create_mca)

        if self.create_catkin:
            installed_ros_distros = os.listdir("/opt/ros")
            click.echo("Available ROS distributions: {}".format(installed_ros_distros))
            self.ros_distro = installed_ros_distros[0]
            if len(installed_ros_distros) > 1:
                ros_distro = click.prompt('Which ROS distribution would you like to use?',
                                          type=click.Choice(installed_ros_distros),
                                          default=installed_ros_distros[0])
            click.echo("Using ROS distribution \'{}\'".format(self.ros_distro))
            if copy_cmake_lists == 'ask':
                copy_cmake_lists = click.confirm("Would you like to copy the top-level "
                                                 "CMakeLists.txt to the catkin"
                                                 " src directory instead of using a symlink?\n"
                                                 "(This is incredibly useful when using the "
                                                 "QtCreator.)",
                                                 default=True)

        click.echo("Creating environment with name \"{}\"".format(self.env_name))

        # Let's get down to business
        self.create_directories()
        self.create_demo_docs()
        self.create_demo_scripts()

        if self.create_ic:
            click.echo("Creating ic_workspace")
            environment_helpers.IcCreator(ic_directory=self.ic_directory,
                                          build_directory=self.ic_build_directory,
                                          rosinstall=self.ic_rosinstall,
                                          packages=self.ic_packages,
                                          package_versions=self.ic_package_versions,
                                          grab_flags=self.ic_grab_flags)
        else:
            click.echo("Requested to not create an ic_workspace")

        # Check if we should create a catkin workspace and create one if desired
        if self.create_catkin:
            click.echo("Creating catkin_ws")

            catkin_creator = \
                environment_helpers.CatkinCreator(catkin_directory=self.catkin_directory,
                                                  build_directory=self.catkin_build_directory,
                                                  rosinstall=self.catkin_rosinstall,
                                                  copy_cmake_lists=copy_cmake_lists)
        else:
            click.echo("Requested to not create a catkin_ws")

        # Check if we should create an mca workspace and create one if desired
        if self.create_mca:
            click.echo("Creating mca_workspace")

            environment_helpers.MCACreator(mca_directory=self.mca_directory,
                                           build_directory=self.mca_build_directory,
                                           mca_additional_repos=self.mca_additional_repos)
        else:
            click.echo("Requested to not create an mca workspace")

        if not no_build:
            if self.create_ic:
                ic_builder = build.IcBuilder(name="ic_builder", add_help_option=False)
                ic_builder.invoke(None)
                self.source_ic_workspace()
            if self.create_catkin:
                ros_builder = build.CatkinBuilder(
                    name=catkin_creator.ros_distro, add_help_option=False)
                ros_builder.invoke(None)
            if self.create_mca:
                mca_builder = build.McaBuilder(name="mca_builder", add_help_option=False)
                mca_builder.invoke(None)

    def create_directories(self):
        """Creates the directory skeleton with build_directories and symlinks"""
        os.mkdir(os.path.join(dir_helpers.get_checkout_dir(), self.env_name))
        os.mkdir(self.demos_dir)

        # Add a custom source file to the environment. Custom source commands go in here.
        env_source_file = open(os.path.join(dir_helpers.get_checkout_dir(),
                                            self.env_name,
                                            "setup_local.sh"),
                               'w')
        env_source_file.write("#This file is for custom source commands in this environment.\n")
        env_source_file.close()

        os.symlink(os.path.join(dir_helpers.get_base_dir(), "bin", "source_environment.sh"),
                   os.path.join(dir_helpers.get_checkout_dir(), self.env_name, "setup.sh"))

    def parse_config(self, config_file):
        """Parses a config file to get an environment information"""
        parser = ConfigFileParser(config_file)

        # Parse ic_workspace packages with error checking
        (self.create_ic,
         self.ic_rosinstall,
         self.ic_packages,
         self.ic_package_versions,
         self.ic_flags) = parser.parse_ic_config()

        # Parse catkin_workspace packages
        self.create_catkin, self.catkin_rosinstall = parser.parse_ros_config()

        # Parse mca_workspace packages
        self.create_mca, self.mca_additional_repos = parser.parse_mca_config()

        # Parse demo scripts and copy them to individual files
        self.script_list = parser.parse_demo_scripts()

    def create_demo_scripts(self):
        """If there are demo scripts given to the environment, create them."""
        click.echo('Found the following demo scripts:')
        for demo in self.script_list:
            click.echo(demo)
            filename = os.path.join(self.demos_dir, demo)
            with open(filename, mode='w') as script_file_content:
                script_file_content.write(self.script_list[demo])
                script_file_content.close()
            os.chmod(filename,
                     stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)

    def source_ic_workspace(self):
        """
        Source the ic_workspace for further environment building
        NOTE: Sourcing this way only works inside the python session and it's children.
        """
        ic_dir = os.path.join(dir_helpers.get_checkout_dir(), self.env_name, "ic_workspace")
        click.echo("Sourcing ic_workspace for environment {}".format(self.env_name))
        lib_path = os.path.join(ic_dir, "export", "lib")
        os.environ['LD_LIBRARY_PATH'] = os.pathsep.join([lib_path,
                                                         os.getenv('LD_LIBRARY_PATH', '')])
        python_path = os.path.join(ic_dir, "export", "lib", "python2.7", "site_packages")
        os.environ['PYTHONPATH'] = os.pathsep.join([python_path, os.getenv('PYTHONPATH', '')])
        path = os.path.join(ic_dir, "export", "bin")
        os.environ['PATH'] = os.pathsep.join([path, os.getenv('PATH', '')])
        qml_import_path = os.path.join(ic_dir, "export", "plugins", "qml")
        os.environ['QML_IMPORT_PATH'] = os.pathsep.join([qml_import_path,
                                                         os.getenv('QML_IMPORT_PATH', '')])
        os.environ['CMAKE_PREFIX_PATH'] = os.pathsep.join([os.path.join(ic_dir, "export"),
                                                           os.getenv('CMAKE_PREFIX_PATH', '')])
        os.environ['IC_MAKER_DIR'] = os.path.join(ic_dir, "icmaker")

    def create_demo_docs(self):
        """Creates a readme.txt in the demos folder to explain it's functionality"""
        doc_filename = os.path.join(self.demos_dir, 'readme.txt')
        with open(doc_filename, 'w') as out_file:
            docstring = '''This folder can contain any executable files. These files can be run
    with the fzirob run command. When scraping an environment to a config file
    all demo scripts will be copied into the environment config, as well.'''
            out_file.write(docstring)


@click.command(short_help='Add a new environment')
@click.option('--config_file', help='Create an environment from a given config file.')
@click.option('--no_build', is_flag=True, default=False,
              help='Do not perform an initial build.')
@click.option('--create_ic', type=click.Choice(['yes', 'no', 'ask']), default='ask',
              help='If set to \'yes\', an ic-workspace is created without asking for it again.')
@click.option('--create_catkin', type=click.Choice(['yes', 'no', 'ask']), default='ask',
              help='If set to \'yes\', a catkin-workspace is created without asking for it again.')
@click.option('--create_mca', type=click.Choice(['yes', 'no', 'ask']), default='ask',
              help='If set to \'yes\', a mca-workspace is created without asking for it again.')
@click.option('--copy_cmake_lists', type=click.Choice(['yes', 'no', 'ask']), default='ask',
              help=('If set to \'yes\', the toplevel CMakeLists files of the catkin workspace is '
                    'copied to the src folder of the catkin ws without asking for it again.'))
@click.option('--local_build', type=click.Choice(['yes', 'no', 'ask']), default='ask',
              help=('If set to \'yes\', the local build options is set and the build is '
                    'executed in the folder ic_workspace/build.'))
@click.argument('env_name', nargs=1)
def cli(env_name,
        config_file,
        no_build,
        create_ic,
        create_catkin,
        create_mca,
        copy_cmake_lists,
        local_build):
    """Adds a new environment and creates the basic needed folders,
    e.g. a ic_workspace and a catkin_ws."""

    environment_creator = EnvCreator(env_name)
    environment_creator.build = not no_build

    os.environ['ROB_FOLDERS_ACTIVE_ENV'] = env_name

    try:
        environment_creator.create_new_environment(config_file,
                                                   no_build,
                                                   create_ic,
                                                   create_catkin,
                                                   create_mca,
                                                   copy_cmake_lists,
                                                   local_build)
        click.echo("Initial workspace setup completed")
    except Exception as my_exception:
        click.echo(my_exception)
        click.echo("Something went wrong while creating the environment!")
