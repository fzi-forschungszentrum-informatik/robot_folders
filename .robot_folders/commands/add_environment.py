import click
import os
import stat
import subprocess
import getpass

from helpers.directory_helpers import get_base_dir
from helpers.directory_helpers import get_checkout_dir
from helpers.directory_helpers import get_catkin_dir
import helpers.build_helpers as build

from yaml import load as yaml_load, dump as yaml_dump

try:
    from yaml import CLoader as Loader, CDumper, Dumper
except ImportError:
    from yaml import Loader, Dumper

# TODO: This file needs some rework. It's become huge and heavy. Make it more modular, please

# NOTE: Sourcing this way only works inside the python session and it's children.
def source_ic_workspace(env_name):
    ic_dir = os.path.join(get_checkout_dir(), env_name, "ic_workspace")
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

    subprocess.call("export", shell=True)

def yes_no_to_bool(str):
    if str == 'yes' or str == 'Yes':
        return True
    else:
        return False

def create_demo_docs(demo_dir):
    doc_filename = os.path.join(demo_dir, 'readme.txt')
    with open(doc_filename, 'w') as file:
        docstring = '''This folder can contain any executable files. These files can be run
with the fzirob run command. When scraping an environment to a config file
all demo scripts will be copied into the environment config, as well.'''
        file.write(docstring)


def create_ic_ws(ic_directory,
                 build_directory,
                 packages=None,
                 package_versions=None,
                 grab_flags=None,
                 rosinstall=None):
    ic_repo_url = "git@ids-git.fzi.de:core/ic_workspace.git"

    if packages is None and rosinstall is None:
        click.error('Either packages or rosinstall must be declared, none was given!')
        return False
    if packages is not None and rosinstall is not None:
        return False
        click.error('Either packages or rosinstall must be declared, not both!')

    click.echo("Creating ic_workspace")
    subprocess.check_call(["git", "clone", ic_repo_url, ic_directory])
    if packages is not None:
        try:
            grab_command = ["./IcWorkspace.py", "grab", packages]
            if grab_flags is not None:
                grab_command.extend(grab_flags)
            process = subprocess.Popen(grab_command, cwd=ic_directory)
            process.wait()

            if package_versions is not None:
                for package in package_versions.keys():
                    if package in packages:
                        click.echo("Checking out version {} of package {}".format(package_versions[package], package))
                        package_dir = os.path.join(ic_directory, "packages", package)
                        click.echo("Package_dir: {}".format(package_dir))
                        process = subprocess.Popen(["git", "checkout", package_versions[package]],
                                                   cwd=package_dir)
                        process.wait()
                    else:
                        click.echo('Version for package {} given, however, package is not listed in environment!'.format(package))


        except subprocess.CalledProcessError as e:
            click.echo(e.output)
            click.echo("An error occurred while creating the ic_workspace. Exiting now")
            return False

    else:
        # if a rosinstall is specified, no base-grabbing is required
        # Dump the rosinstall to a file and use wstool for getting the packages
        rosinstall_filename = '/tmp/rob_folders_rosinstall'
        rosinstall_file_handle = file(rosinstall_filename, 'w')
        yaml_dump(rosinstall, rosinstall_file_handle)
        try:
            process = subprocess.Popen(["wstool", "init", "packages", rosinstall_filename],
                                       cwd=ic_directory)
            process.wait()
            os.remove(rosinstall_filename)

            # It is necessary to grab the base packages to get an icmaker
            grab_command = ["./IcWorkspace.py", "grab", "base"]
            process = subprocess.Popen(grab_command, cwd=ic_directory)
            process.wait()

        except subprocess.CalledProcessError as e:
            click.echo(e.output)
            click.echo("An error occurred while creating the ic_workspace. Exiting now")
            return False


    # Create the build and export directories
    os.makedirs(build_directory)
    (export_base_dir, _) = os.path.split(build_directory)
    export_directory = os.path.join(export_base_dir, "export")
    os.makedirs(export_directory)

    # Check if we need symlinks to the build and export directories and create them
    local_build_dir_name = os.path.join(ic_directory, "build")
    local_export_dir_name = os.path.join(ic_directory, "export")
    if local_build_dir_name != build_directory:
        os.symlink(build_directory, local_build_dir_name)
    if local_export_dir_name != export_directory:
        os.symlink(export_directory, local_export_dir_name)

    return True


@click.command(short_help='Add a new environment')
@click.option('--config_file', help='Create an environment from a given config file.')
@click.option('--no_build', is_flag=True, default=False,
              help='Do not perform an initial build.')
@click.option('--create_ic', type=click.Choice(['yes', 'no', 'ask']), default='ask', help='If set to \'yes\', an ic-workspace is created without asking for it again.')
@click.option('--create_catkin', type=click.Choice(['yes', 'no', 'ask']), default='ask', help='If set to \'yes\', a catkin-workspace is created without asking for it again.')
@click.option('--create_mca', type=click.Choice(['yes', 'no', 'ask']), default='ask', help='If set to \'yes\', a mca-workspace is created without asking for it again.')
@click.option('--copy_cmake_lists', type=click.Choice(['yes', 'no', 'ask']), default='ask', help='If set to \'yes\', the toplevel CMakeLists files of the catkin workspace is copied to the src folder of the catkin ws without asking for it again.')
@click.option('--local_build', type=click.Choice(['yes', 'no', 'ask']), default='ask', help='If set to \'yes\', the local build options is set and the build is executed in the folder ic_workspace/build.')
@click.argument('env_name', nargs=1)
#@click.option('-m', '--mca2_workspace', default=False, help='Create an mac2_workspace.')
def cli(env_name, config_file, no_build, create_ic, create_catkin, create_mca, copy_cmake_lists, local_build):
    """Adds a new environment and creates the basic needed folders, e.g. a ic_orkspace and a catkin_ws."""

    base_dir = get_base_dir()
    build_base_dir = get_checkout_dir()
    ic_packages = "base"
    ic_package_versions = {}
    ic_grab_flags = []
    ic_cmake_flags = ""
    ic_rosinstall = None
    cama_flags = ""
    catkin_rosinstall = ""
    mca_repo_url = "git@ids-git.fzi.de:mca2/mca2.git"
    mca_cmake_flags = ""
    mca_additional_repos = ""

    os.environ['ROB_FOLDERS_ACTIVE_ENV'] = env_name
    

    # If we are on a workstation or when no_backup is mounted like on a workstation offer to build in no_backup
    has_nobackup = False
    try:
        if os.path.isdir('/disk/no_backup'):
            has_nobackup = True
    except subprocess.CalledProcessError as e:
        pass

    if has_nobackup:
        if local_build== 'ask':
            build_dir_choice = click.prompt("Which folder should I use as a base for creating the build tree?\nType 'local' for building inside the local robot_folders tree.\nType 'no_backup' (or simply press enter) for building in the no_backup space (should be used on workstations).\n",
                                        type=click.Choice(['no_backup', 'local']),
                                        default='no_backup')
        else:
            build_dir_choice = local_build

        if build_dir_choice == 'no_backup':
            username = getpass.getuser()
            build_base_dir = '/disk/no_backup/{}/robot_folders_build_base'.format(username)

    if os.path.exists(os.path.join(get_checkout_dir(), env_name)):
        click.echo("An environment with the name \"{}\" already exists. Exiting now.".format(env_name))
        return

    # Set all necessary paths for the environments.
    click.echo("Creating environment with name \"{}\"".format(env_name))
    os.mkdir(os.path.join(get_checkout_dir(), env_name))
    demos_dir = os.path.join(get_checkout_dir(), env_name, 'demos')
    os.mkdir(demos_dir)
    create_demo_docs(demos_dir)
    ic_directory = os.path.join(get_checkout_dir(), env_name, "ic_workspace")
    ic_build_directory = os.path.join(build_base_dir, env_name,  "ic_workspace", "build")
    catkin_directory = os.path.join(get_checkout_dir(), env_name, "catkin_ws")
    catkin_build_directory = os.path.join(build_base_dir, env_name, "catkin_ws", "build")
    mca_directory = os.path.join(get_checkout_dir(), env_name, "mca_workspace")
    mca_build_directory = os.path.join(build_base_dir, env_name, "mca_workspace", "build")

    # If we have a config file check which packages should be fetched.
    # Otherwise ask which empty workspaces should be created
    if config_file:
        yaml_stream = file(config_file, 'r')
        data = yaml_load(yaml_stream, Loader=Loader)
        click.echo(data)

        # Parse ic_workspace packages with error checking
        if 'ic_workspace' in data:
            create_ic = True
            if 'rosinstall' in data['ic_workspace']:
                ic_rosinstall = data['ic_workspace']['rosinstall']
                ic_packages = None

            if 'packages' in data['ic_workspace']:
                ic_packages = ' '.join(data['ic_workspace']['packages'])
                # if the base package is not specified in the config_file, add it nevertheless.
                #TODO: alternatively only print the warning
                if "base" not in ic_packages:
                    ic_packages = "base " + ic_packages
                    click.echo("The base-package for the ic_workspace has not been specified in the"
                               " configuration. It will be added regardless.")
                if 'package_versions' in data['ic_workspace'] and data['ic_workspace']['package_versions'] is not None:
                    ic_package_versions = data['ic_workspace']['package_versions']
                if 'flags' in data['ic_workspace'] and data['ic_workspace']['flags'] is not None:
                    for flag in data['ic_workspace']['flags']:
                        ic_grab_flags.append("--" + flag)

                ic_rosinstall = None

        if 'catkin_workspace' in data:
            create_catkin = True
            if data['catkin_workspace'] is not None and 'rosinstall' in data['catkin_workspace']:
                if data['catkin_workspace']['rosinstall'] is not None:
                    catkin_rosinstall = data['catkin_workspace']['rosinstall']

        if 'mca_workspace' in data:
            create_mca = True
            if data['mca_workspace'] is not None:
                mca_additional_repos = data['mca_workspace']

        if 'demos' in data:
            if data['demos'] is not None:
                print ('Found the following demo scripts:')
                for demo in data['demos']:
                    print(demo)
                    filename = os.path.join(get_checkout_dir(), env_name, 'demos', demo)
                    with open(filename, mode='w') as f:
                        f.write(data['demos'][demo])
                        f.close()
                    os.chmod(filename, 00755)

    # Check given flags or ask user
    else:
        if create_ic== 'ask':
            create_ic = click.confirm("Would you like to create an ic_workspace?", default=True)
        else:
            create_ic = yes_no_to_bool(create_ic)

        if create_catkin == 'ask':
            create_catkin = click.confirm("Would you like to create a catkin_ws?", default=True)
        else:
            create_catkin = yes_no_to_bool(create_catkin)

        if create_mca == 'ask':
            create_mca = click.confirm("Would you like to create an mca_workspace?", default=True)
        else:
            create_mca = yes_no_to_bool(create_mca)

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
        ros_global_dir = "/opt/ros/{}".format(ros_distro)


        if copy_cmake_lists == 'ask':
            copy_cmake_lists = click.confirm(("Would you like to copy the top-level CMakeLists.txt to the catkin"
                                         " src directory instead of using a symlink?\n"
                                         "(This is incredibly useful when using the QtCreator.)"),
                                         default=True)
        else:
            copy_cmake_lists = yes_no_to_bool(copy_cmake_lists)

    # Add a custom source file to the environment. Custom source commands go in here.
    env_source_file = open(os.path.join(get_checkout_dir(), env_name, "setup_local.sh"), 'w')
    env_source_file.write("#This file is for custom source commands in this environment.\n")
    env_source_file.close()

    os.symlink(os.path.join(get_base_dir(), "bin", "source_environment.sh"),
               os.path.join(get_checkout_dir(), env_name, "setup.sh"))


    # Now, we're done asking the user. Let's get to work
    if create_ic:
        create_ic_ws(ic_directory=ic_directory,
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

        # Create directories and symlinks, if necessary
        os.mkdir(catkin_directory)
        os.mkdir(os.path.join(catkin_directory, "src"))
        os.makedirs(catkin_build_directory)

        local_build_dir_name = os.path.join(catkin_directory, "build")
        (catkin_base_dir, _) = os.path.split(catkin_build_directory)

        catkin_devel_directory = os.path.join(catkin_base_dir, "devel")
        local_devel_dir_name = os.path.join(catkin_directory, "devel")
        click.echo("devel_dir: {}".format(catkin_devel_directory))

        catkin_install_directory = os.path.join(catkin_base_dir, "install")
        local_install_dir_name = os.path.join(catkin_directory, "install")
        click.echo("install_dir: {}".format(catkin_install_directory))

        if local_build_dir_name != catkin_build_directory:
            os.symlink(catkin_build_directory, local_build_dir_name)
            os.makedirs(catkin_devel_directory)
            os.symlink(catkin_devel_directory, local_devel_dir_name)
            os.makedirs(catkin_install_directory)
            os.symlink(catkin_install_directory, local_install_dir_name)

        # We abuse the name parameter to code the ros distribution
        # if we're building for the first time.
        ros_builder = build.CatkinBuilder(name=ros_distro,
                                          add_help_option=False)
        ros_builder.invoke(None)

        # copy packages
        if catkin_rosinstall != "":
            # Dump the rosinstall to a file and use wstool for getting the packages
            rosinstall_filename = '/tmp/rob_folders_rosinstall'
            rosinstall_file_handle = file(rosinstall_filename, 'w')
            yaml_dump(catkin_rosinstall, rosinstall_file_handle)
            process = subprocess.Popen(["wstool", "init", "src", rosinstall_filename],
                                       cwd=catkin_directory)
            process.wait()
            os.remove(rosinstall_filename)

        if copy_cmake_lists:
            try:
                subprocess.check_call(["rm", "{}/src/CMakeLists.txt".format(catkin_directory)])
                subprocess.check_call(["cp",
                                       "{}/share/catkin/cmake/toplevel.cmake".format(ros_global_dir),
                                       "{}/src/CMakeLists.txt".format(catkin_directory)])
            except subprocess.CalledProcessError as e:
                click.echo(e.output)
                click.echo("An error occurred while copying the CMakeLists.txt to the catkin source directory")
    else:
        click.echo("Requested to not create a catkin_ws")

    # Check if we should create an mca workspace and create one if desired
    if create_mca:
        click.echo("Creating mca_workspace")

        try:
            subprocess.check_call(["git", "clone", mca_repo_url, mca_directory])

            if 'libraries' in mca_additional_repos and mca_additional_repos['libraries'] is not None:
                for library in mca_additional_repos['libraries']:
                    libraries_dir = os.path.join(mca_directory,
                                                 'libraries',
                                                 library['git']['local-name'])
                    subprocess.check_call(["git", "clone", library['git']['uri'], libraries_dir])
                    if 'version' in library['git']:
                        package_version = library['git']['version']
                        click.echo("Checking out version {} of package {}".format(package_version, library['git']['local-name']))
                        process = subprocess.Popen(["git", "checkout", package_version],
                                                    cwd=libraries_dir)
                        process.wait()

            if 'projects' in mca_additional_repos and mca_additional_repos['projects'] is not None:
                for project in mca_additional_repos['projects']:
                    click.echo("Project: {}".format(project))
                    projects_dir = os.path.join(mca_directory,
                                                'projects',
                                                project['git']['local-name'])
                    subprocess.check_call(["git", "clone", project['git']['uri'], projects_dir])
                    if 'version' in project['git']:
                        package_version = project['git']['version']
                        click.echo("Checking out version {} of package {}".format(package_version, project['git']['local-name']))
                        process = subprocess.Popen(["git", "checkout", package_version],
                                                    cwd=projects_dir)
                        process.wait()

            if 'tools' in mca_additional_repos and mca_additional_repos['tools'] is not None:
                for tool in mca_additional_repos['tools']:
                    tools_dir = os.path.join(mca_directory,
                                             'tools',
                                             tool['git']['local-name'])
                    subprocess.check_call(["git", "clone", tool['git']['uri'], tools_dir])
                    if 'version' in tool['git']:
                        package_version = tool['git']['version']
                        click.echo("Checking out version {} of package {}".format(package_version, tool['git']['local-name']))
                        process = subprocess.Popen(["git", "checkout", package_version],
                                                    cwd=tools_dir)
                        process.wait()

            process = subprocess.Popen(["script/git_clone_base.py"],
                                       cwd=mca_directory)
            process.wait()

            process = subprocess.Popen(["script/ic/update_cmake.py"],
                                       cwd=mca_directory)
            process.wait()

            os.makedirs(mca_build_directory)
            local_build_dir_name = os.path.join(mca_directory, "build")
            if local_build_dir_name != mca_build_directory:
                os.symlink(mca_build_directory, local_build_dir_name)

        except subprocess.CalledProcessError as e:
            click.echo(e.output)
            click.echo("An error occurred while creating the mca_workspace. Exiting now")
            return
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
