import click
import os
import stat
import subprocess
import getpass

from helpers.directory_helpers import get_base_dir
from helpers.directory_helpers import get_checkout_dir
from helpers.directory_helpers import get_catkin_dir
import helpers.build_helpers as build
import helpers.environment_helpers as environment_helpers
from helpers.ConfigFileParser import ConfigFileParser

from yaml import load as yaml_load, dump as yaml_dump

try:
    from yaml import CLoader as Loader, CDumper, Dumper
except ImportError:
    from yaml import Loader, Dumper


class EnvCreator:
    __init__(self, name):
        self.env_name = name
        self.build_base_dir = get_checkout_dir()
        self.ic_packages = "base"
        self.ic_package_versions = {}
        self.ic_grab_flags = []
        self.ic_cmake_flags = ""
        self.ic_rosinstall = None
        self.cama_flags = ""
        self.catkin_rosinstall = ""
        self.mca_cmake_flags = ""
        self.mca_additional_repos = ""
        self.script_list = list()
        self.build = True
        self.has_nobackup = False

        os.environ['ROB_FOLDERS_ACTIVE_ENV'] = env_name

        self.create_ic = False
        self.create_catkin = False
        self.create_mca = False

    def create_new_environment(self, env_name, config_file=None):
        if os.path.exists(os.path.join(get_checkout_dir(), env_name)):
            click.echo("An environment with the name \"{}\" already exists. Exiting now.".format(env_name))
            return
        self.check_nobackup()

        self.demos_dir = os.path.join(get_checkout_dir(), self.env_name, 'demos')

        # Build directories
        self.ic_directory = os.path.join(get_checkout_dir(), self.env_name, "ic_workspace")
        self.ic_build_directory = os.path.join(self.build_base_dir, self.env_name,  "ic_workspace", "build")
        self.catkin_directory = os.path.join(get_checkout_dir(), self.env_name, "catkin_ws")
        self.catkin_build_directory = os.path.join(self.build_base_dir, self.env_name, "catkin_ws", "build")
        self.mca_directory = os.path.join(get_checkout_dir(), self.env_name, "mca_workspace")
        self.mca_build_directory = os.path.join(self.build_base_dir, self.env_name, "mca_workspace", "build")

        click.echo("Creating environment with name \"{}\"".format(env_name))
        self.create_directories()
        self.create_demo_docs(demos_dir)

    def check_nobackup(self):
        # If we are on a workstation or when no_backup is mounted like on a workstation offer to build in no_backup
        has_nobackup = False
        try:
            if os.path.isdir('/disk/no_backup'):
                has_nobackup = True
        except subprocess.CalledProcessError as e:
            pass

        if has_nobackup:
            build_dir_choice = click.prompt("Which folder should I use as a base for creating the build tree?\nType 'local' for building inside the local robot_folders tree.\nType 'no_backup' (or simply press enter) for building in the no_backup space (should be used on workstations).\n",
                                            type=click.Choice(['no_backup', 'local']),
                                            default='no_backup')
            if build_dir_choice == 'no_backup':
                username = getpass.getuser()
                self.build_base_dir = '/disk/no_backup/{}/robot_folders_build_base'.format(username)

    def create_directories(self):
        os.mkdir(os.path.join(get_checkout_dir(), self.env_name))
        os.mkdir(self.demos_dir)

    def parse_config(self):
        parser = ConfigFileParser(config_file)

        # Parse ic_workspace packages with error checking
        self.create_ic, self.ic_rosinstall, self.ic_packages, self.ic_package_versions, self.ic_flags = \
            parser.parse_ic_config()
        # Parse catkin_workspace packages
        self.create_catkin, self.catkin_rosinstall = parser.parse_ros_config()

        # Parse mca_workspace packages
        self.create_mca, self.mca_additional_repos = parser.parse_mca_config()

        # Parse demo scripts and copy them to individual files
        self.script_list = parser.parse_demo_scripts()

    def create_demo_scripts(self)
        click.echo('Found the following demo scripts:')
        for demo in script_list:
            click.echo(demo)
            filename = os.path.join(self.demos_dir, demo)
            with open(filename, mode='w') as f:
                f.write(self.script_list[demo])
                f.close()
            os.chmod(filename, 00755)


    # NOTE: Sourcing this way only works inside the python session and it's children.
    def source_ic_workspace(self):
        ic_dir = os.path.join(get_checkout_dir(), self.env_name, "ic_workspace")
        click.echo("Sourcing ic_workspace for environment {}".format(env_name))
        lib_path = os.path.join(ic_dir, "export", "lib")
        os.environ['LD_LIBRARY_PATH'] = os.pathsep.join([lib_path, os.getenv('LD_LIBRARY_PATH', '')])
        python_path = os.path.join(ic_dir, "export", "lib", "python2.7", "site_packages")
        os.environ['PYTHONPATH'] = os.pathsep.join([python_path, os.getenv('PYTHONPATH', '')])
        path = os.path.join(ic_dir, "export", "bin")
        os.environ['PATH'] = os.pathsep.join([path, os.getenv('PATH', '')])
        qml_import_path = os.path.join(ic_dir, "export", "plugins", "qml")
        os.environ['QML_IMPORT_PATH'] = os.pathsep.join([qml_import_path, os.getenv('QML_IMPORT_PATH', '')])
        os.environ['CMAKE_PREFIX_PATH'] = os.pathsep.join([os.path.join(ic_dir, "export"), os.getenv('CMAKE_PREFIX_PATH', '')])
        os.environ['IC_MAKER_DIR'] = os.path.join(ic_dir, "icmaker")

    def create_demo_docs(self):
        doc_filename = os.path.join(self.demos_dir, 'readme.txt')
        with open(doc_filename, 'w') as file:
            docstring = '''This folder can contain any executable files. These files can be run
    with the fzirob run command. When scraping an environment to a config file
    all demo scripts will be copied into the environment config, as well.'''
            file.write(docstring)


@click.command(short_help='Add a new environment')
@click.option('--config_file', help='Create an environment from a given config file.')
@click.option('--no_build', is_flag=True, default=False,
              help='Do not perform an initial build.')
@click.argument('env_name', nargs=1)
def cli(env_name, config_file, no_build):
    """Adds a new environment and creates the basic needed folders, e.g. a ic_orkspace and a catkin_ws."""

    environment_creator = EnvCreator(env_name)
    environment_creator.build = not no_build

    # If we have a config file check which packages should be fetched.
    # Otherwise ask which empty workspaces should be created
    if config_file:
        self.parse_config()
    else:
        create_ic = click.confirm("Would you like to create an ic_workspace?", default=True)
        create_catkin = click.confirm("Would you like to create a catkin_ws?", default=True)
        create_mca = click.confirm("Would you like to create an mca_workspace?", default=True)

    # If we create a catkin_ws query some more stuff at the beginning of the script.
    if create_catkin:
        installed_ros_distros = os.listdir("/opt/ros")
        click.echo("Available ROS distributions: {}".format(installed_ros_distros))
        ros_distro = installed_ros_distros[0]
        if len(installed_ros_distros) > 1:
            ros_distro = click.prompt('Which ROS distribution would you like to use?',
                                      type=click.Choice(installed_ros_distros),
                                      default=installed_ros_distros[0])
        click.echo("Using ROS distribution \'{}\'".format(ros_distro))
        copy_cmake_lists = click.confirm(("Would you like to copy the top-level CMakeLists.txt to the catkin"
                                         " src directory instead of using a symlink?\n"
                                         "(This is incredibly useful when using the QtCreator.)"),
                                         default=True)

    # Add a custom source file to the environment. Custom source commands go in here.
    env_source_file = open(os.path.join(get_checkout_dir(), env_name, "setup_local.sh"), 'w')
    env_source_file.write("#This file is for custom source commands in this environment.\n")
    env_source_file.close()

    os.symlink(os.path.join(get_base_dir(), "bin", "source_environment.sh"),
               os.path.join(get_checkout_dir(), env_name, "setup.sh"))


    # Now, we're done asking the user. Let's get to work

    # Check if we should create a catkin workspace and create one if desired
    if create_ic:
        click.echo("Creating ic_workspace")
        environment_helpers.IcCreator(ic_directory=ic_directory,
                                      build_directory=ic_build_directory, 
                                      rosinstall=ic_rosinstall,
                                      packages=ic_packages,
                                      package_versions=ic_package_versions,
                                      grab_flags=ic_grab_flags)
    else:
        click.echo("Requested to not create an ic_workspace")

    # Check if we should create a catkin workspace and create one if desired
    if create_catkin:
        click.echo("Creating catkin_ws")

        environment_helpers.CatkinCreator(catkin_directory=catkin_directory,
                                          build_directory=catkin_build_directory,
                                          ros_distro=ros_distro,
                                          copy_cmake_lists=copy_cmake_lists)
    else:
        click.echo("Requested to not create a catkin_ws")

    # Check if we should create an mca workspace and create one if desired
    if create_mca:
        click.echo("Creating mca_workspace")

        environment_helpers.MCACreator(mca_directory, mca_build_directory, mca_additional_repos)
    else:
        click.echo("Requested to not create an mca workspace")

    # let's build if requested
    os.environ['ROB_FOLDERS_ACTIVE_ENV'] = env_name
    if not no_build:
        if create_ic:
            ic_builder = build.IcBuilder(name="ic_builder", add_help_option=False)
            ic_builder.invoke(None)
            source_ic_workspace(env_name)
        if create_catkin:
            ros_builder = build.CatkinBuilder(name=ros_distro, add_help_option=False)
            ros_builder.invoke(None)
        if create_mca:
            mca_builder = build.McaBuilder(name="mca_builder", add_help_option=False)
            mca_builder.invoke(None)

    click.echo("Initial workspace setup completed")
