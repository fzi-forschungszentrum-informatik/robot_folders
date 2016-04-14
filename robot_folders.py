import click
import os
import subprocess
import getpass

from yaml import load as yaml_load, dump as yaml_dump

try:
    from yaml import CLoader as Loader, CDumper, Dumper
except ImportError:
    from yaml import Loader, Dumper

def get_base_dir():
    return os.path.dirname(os.path.realpath(__file__))

def get_active_env():
    env_file = os.path.join(get_base_dir(), 'checkout', '') + '.cur_env'
    if os.path.isfile(env_file):
        with open(env_file, 'r') as file:
            return file.read().rstrip()
    else:
        return None

def get_active_env_path():
    if get_active_env() == None:
        return None
    return os.path.join(get_base_dir(), 'checkout', get_active_env())

@click.group()
def cli():
    """A simple command line tool."""

# NOTE: Sourcing this way only works inside the python session and it's children.
def source_ic_workspace(env_name):
    ic_dir = os.path.join(get_base_dir(), "checkout", env_name, "ic_workspace")
    click.echo("Sourcing ic_workspace for environment {}".format(env_name))
    lib_path = os.path.join(ic_dir, "export", "lib")
    os.environ['LD_LIBRARY_PATH'] = os.pathsep.join([lib_path, os.environ['LD_LIBRARY_PATH']])
    python_path = os.path.join(ic_dir, "export", "lib", "python2.7", "site_packages")
    os.environ['PYTHONPATH'] = os.pathsep.join([python_path, os.environ.get('PYTHONPATH', '')])
    path = os.path.join(ic_dir, "export", "bin")
    os.environ['PATH'] = os.pathsep.join([path, os.environ['PATH']])
    qml_import_path = os.path.join(ic_dir, "export", "plugins", "qml")
    os.environ['QML_IMPORT_PATH'] = os.pathsep.join([qml_import_path, os.environ.get('QML_IMPORT_PATH', '')])
    os.environ['CMAKE_PREFIX_PATH'] = os.path.join(ic_dir, "export")
    os.environ['IC_MAKER_DIR'] = os.path.join(ic_dir, "icmaker")

    subprocess.call("export", shell=True)


@cli.command('add_environment', short_help='Add a new environment')
@click.option('--generator', type=click.Choice(['ninja', 'makefiles']),
              default='ninja',
              help='Which generator should be used in this environment? Defaults to ninja.')
@click.option('--config_file', help='Create an environment from a given config file.')
@click.option('--no_build', is_flag=True, default=False,
              help='Do not perform an initial build. A cmake run will be performed anyway. For catkin workspaces, a full catkin_make will be performed afterwards.')
@click.option('--no_cmake', is_flag=True, default=False,
              help='Only checkout the repositories. The build process has to be done manually afterwards. Please be aware, that sourcing such an environment is not directly possible.')
@click.argument('env_name', nargs=1)
#@click.option('-m', '--mca2_workspace', default=False, help='Create an mac2_workspace.')
def add_environment(generator, env_name, config_file, no_build, no_cmake):
    """Adds a new environment and creates the basic needed folders, e.g. a ic_orkspace and a catkin_workspace."""

    base_dir = get_base_dir()
    build_base_dir = os.path.join(base_dir, "checkout")
    ic_repo_url = "git://idsgit.fzi.de/core/ic_workspace.git"
    ic_packages = "base"
    ic_package_versions = {}
    ic_cmake_flags = ""
    cama_flags = ""
    catkin_rosinstall = ""
    mca_repo_url = "git://idsgit.fzi.de/mca2/mca2.git"
    mca_cmake_flags = ""
    mca_additional_repos = ""
    build_cmd = "make install"

    create_ic = False
    create_catkin = False
    create_mca = False


    # If we are on a workstation or when no_backup is mounted like on a workstation offer to build in no_backup
    has_nobackup = False
    try:
        if os.path.isdir('/disk/no_backup'):
            has_nobackup = True
    except subprocess.CalledProcessError as e:
        pass

    if has_nobackup:
        if not (no_cmake):
            build_dir_choice = click.prompt("Which folder should I use as a base for creating the build tree?\nType 'local' for building inside the local robot_folders tree.\nType 'no_backup' (or simply press enter) for building in the no_backup space (should be used on workstations).\n",
                                            type=click.Choice(['no_backup', 'local']),
                                            default='no_backup')
            if build_dir_choice == 'no_backup':
                username = getpass.getuser()
                build_base_dir = '/disk/no_backup/{}/robot_folders_build_base'.format(username)



    if config_file:
        yaml_stream = file(config_file, 'r')
        data = yaml_load(yaml_stream, Loader=Loader)
        click.echo(data)

        # Parse ic_workspace packages with error checking
        if 'ic_workspace' in data:
            create_ic = True
            if data['ic_workspace'] is not None and 'packages' in data['ic_workspace']:
                if data['ic_workspace']['packages'] is not None:
                    ic_packages = ' '.join(data['ic_workspace']['packages'])
                    ic_package_versions = data['ic_workspace']['package_versions']


        if 'catkin_workspace' in data:
            create_catkin = True
            if data['catkin_workspace'] is not None and 'rosinstall' in data['catkin_workspace']:
                if data['catkin_workspace']['rosinstall'] is not None:
                    catkin_rosinstall = data['catkin_workspace']['rosinstall']

        if 'mca_workspace' in data:
            create_mca = True
            if data['mca_workspace'] is not None:
                mca_additional_repos = data['mca_workspace']
    else:
        create_ic = click.confirm("Would you like to create an ic_workspace?", default=True)
        create_catkin = click.confirm("Would you like to create a catkin_workspace?", default=True)
        create_mca = click.confirm("Would you like to create an mca_workspace?", default=True)

    # If we create a catkin_workspace query some more stuff at the beginning of the script.
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
        copy_cmake_lists = click.confirm(("Would you like to copy the top-level CMakeLists.txt to the catkin"
                                        " src directory instead of using a symlink?\n"
                                        "(This is incredibly useful when using the QtCreator.)"),
                                        default=True)






    if generator == 'ninja':
        sep = " "
        ic_cmake_flags = sep.join([ic_cmake_flags, "-GNinja"])
        cama_flags = sep.join([cama_flags, "--use-ninja"])
        mca_cmake_flags = sep.join([mca_cmake_flags, "-GNinja"])
        build_cmd = "ninja  install"


    if os.path.exists("{}/checkout/{}".format(base_dir, env_name)):
        click.echo("An environment with the name \"{}\" already exists. Exiting now.".format(env_name))
        return

    click.echo("Creating environment with name \"{}\"".format(env_name))
    os.mkdir("{}/checkout/{}".format(base_dir, env_name))
    ic_directory = os.path.join(base_dir, "checkout", env_name, "ic_workspace")
    ic_build_directory = os.path.join(build_base_dir, env_name,  "ic_workspace", "build")
    catkin_directory = os.path.join(base_dir, "checkout", env_name, "catkin_workspace")
    catkin_build_directory = os.path.join(build_base_dir, env_name, "catkin_workspace", "build")

    mca_directory = os.path.join(base_dir, "checkout", env_name, "mca_workspace")
    mca_build_directory = os.path.join(build_base_dir, env_name, "mca_workspace", "build")


    env_source_file = open(os.path.join(base_dir, "checkout", env_name, "source_local.sh"), 'w')


    env_source_file.write("#This file is for custom source commands in this environment.\n")
    env_source_file.close()


    # Check if we should create an ic workspace and create one if desired
    if create_ic:
        click.echo("Creating ic_workspace")

        try:
            subprocess.check_call(["git", "clone", ic_repo_url, ic_directory])

            process = subprocess.Popen(["./IcWorkspace.py", "grab", ic_packages],
                                       cwd=ic_directory)
            process.wait()

            for package in ic_package_versions.keys():
                if package in ic_packages:
                    click.echo("Checking out version {} of package {}".format(ic_package_versions[package], package))
                    package_dir = os.path.join(ic_directory, "packages", package)
                    click.echo("Package_dir: {}".format(package_dir))
                    process = subprocess.Popen(["git", "checkout", ic_package_versions[package]],
                                                cwd=package_dir)
                    process.wait()
                else:
                    click.echo('Version for package {} given, however, package is not listed in environment!'.format(package))

            os.makedirs(ic_build_directory)
            local_build_dir_name = os.path.join(ic_directory, "build")
            if local_build_dir_name != ic_build_directory:
                os.symlink(ic_build_directory, local_build_dir_name)
            if not no_cmake:
                ic_cmake_cmd = "cmake {} {}".format(ic_directory, ic_cmake_flags)
                process = subprocess.Popen(["bash", "-c", ic_cmake_cmd],
                                           cwd=ic_build_directory)
                process.wait()
                if not no_build:
                    process = subprocess.Popen(["bash", "-c", build_cmd],
                                               cwd=ic_build_directory)
                    process.wait()
                    source_ic_workspace(env_name)

        except subprocess.CalledProcessError as e:
            click.echo(e.output)
            click.echo("An error occurred while creating the ic_workspace. Exiting now")
            return

    else:
        click.echo("Requested to not create an IC Workspace")


    # Check if we should create a catkin workspace and create one if desired
    if create_catkin:
        click.echo("Creating catkin_workspace")

        os.mkdir(catkin_directory)
        if catkin_rosinstall == "":
            os.mkdir(os.path.join(catkin_directory, "src"))
        else:
            # Dump the rosinstall to a file and use wstool for getting the packages
            rosinstall_filename = '/tmp/rob_folders_rosinstall'
            rosinstall_file_handle = file(rosinstall_filename, 'w')
            yaml_dump(catkin_rosinstall, rosinstall_file_handle)
            process = subprocess.Popen(["wstool", "init", "src", rosinstall_filename],
                                   cwd=catkin_directory)
            process.wait()

        os.makedirs(catkin_build_directory)
        local_build_dir_name = os.path.join(catkin_directory, "build")
        (catkin_devel_base_dir, _) = os.path.split(catkin_build_directory)
        catkin_devel_directory = os.path.join(catkin_devel_base_dir, "devel")
        local_devel_dir_name = os.path.join(catkin_directory, "devel")
        click.echo("devel_dir: {}".format(catkin_devel_directory))
        if local_build_dir_name != catkin_build_directory:
            os.symlink(catkin_build_directory, local_build_dir_name)
            os.makedirs(catkin_devel_directory)
            os.symlink(catkin_devel_directory, local_devel_dir_name)
        # Perform catkin_make only if neither of no_cmake and no_build is declared
        if not (no_cmake or no_build):
            cama_command = "source {}/setup.bash && catkin_make {}".format(ros_global_dir, cama_flags)

            process = subprocess.Popen(["bash", "-c", cama_command],
                                       cwd=catkin_directory)
            process.wait()

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
        click.echo("Requested to not create a catkin_workspace")




    # Check if we should create an mca workspace and create one if desired
    if create_mca:
        click.echo("Creating mca_workspace")

        try:
            subprocess.check_call(["git", "clone", mca_repo_url, mca_directory])

            process = subprocess.Popen(["script/git_clone_base.py"],
                                       cwd=mca_directory)
            process.wait()

            process = subprocess.Popen(["script/ic/update_cmake.py"],
                                       cwd=mca_directory)
            process.wait()

            if 'projects' in mca_additional_repos and mca_additional_repos['projects'] is not None:
                for project in mca_additional_repos['projects']:
                    click.echo("Project: {}".format(project))
                    projects_dir = os.path.join(mca_directory,
                                                'projects',
                                                project['git']['local-name'])
                    subprocess.check_call(["git", "clone", project['git']['uri'], projects_dir])

            if 'libraries' in mca_additional_repos and mca_additional_repos['libraries'] is not None:
                for library in mca_additional_repos['libraries']:
                    libraries_dir = os.path.join(mca_directory,
                                                 'libraries',
                                                 library['git']['local-name'])
                    subprocess.check_call(["git", "clone", library['git']['uri'], libraries_dir])

            os.makedirs(mca_build_directory)
            local_build_dir_name = os.path.join(mca_directory, "build")
            if local_build_dir_name != mca_build_directory:
                os.symlink(mca_build_directory, local_build_dir_name)
            if not no_cmake:
                mca_cmake_cmd = "cmake {} {}".format(mca_directory, mca_cmake_flags)
                process = subprocess.Popen(["bash", "-c", mca_cmake_cmd],
                                           cwd=mca_build_directory)
                if not no_build:
                    process.wait()
                    process = subprocess.Popen(["bash", "-c", build_cmd],
                                               cwd=mca_build_directory)
                    process.wait()

        except subprocess.CalledProcessError as e:
            click.echo(e.output)
            click.echo("An error occurred while creating the ic_workspace. Exiting now")
            return
    else:
        click.echo("Requested to not create an mca workspace")


        click.echo("Initial workspace setup completed")


@cli.command('active_environment')
def active_environment():
    """Prints out the current environment."""
    click.echo("Active environment: {}".format(get_active_env()))


class WorkspaceChooser(click.MultiCommand):
    def get_workspaces(self):
        checkout_folder = os.path.join(get_base_dir(), 'checkout')
        # TODO possibly check whether the directory contains actual workspace
        return [dir for dir in os.listdir(checkout_folder) if os.path.isdir(os.path.join(checkout_folder, dir))]

    def list_commands(self, ctx):
        if get_active_env_path() == None:
            return []
        workspaces = [dir for dir in os.listdir(get_active_env_path())]
        cmds = []
        if 'ic_workspace' in workspaces:
            cmds.append('ic')
        if 'mca_workspace' in workspaces:
            cmds.append('mca')
        if 'catkin_workspace' in workspaces:
            cmds.append('ros')

        return cmds

    def format_commands(self, ctx, formatter):
        return 'ic, ros'
        pass

    def get_command(self, ctx, name):
        click.echo(ctx.params)
        return self


@cli.command('make', cls=WorkspaceChooser, invoke_without_command=True)
@click.pass_context
def make(ctx):
    """Changes the global environment to the specified one. All environment-specific commands are then executed relative to that environment. \
       Only one environment can be active at a time."""
    click.echo('call!')
    click.echo(ctx.params)
    click.echo(ctx.invoked_subcommand)
    exit(0)
    if ctx.invoked_subcommand is None:
        click.echo("make called without argument")


@cli.command('delete', short_help='delete the repo')
def delete():
    """Deletes the repository."""
    pass

class EnvironmentChoice(click.Command):
    def invoke(self, ctx):
        with open( os.path.join(get_base_dir(), 'checkout', '') + '.cur_env', 'w') as file:
            file.write("{}".format(self.name))

        with open( os.path.join(get_base_dir(), 'checkout', '.source_cur_env'), 'w') as file:
            main_source = os.path.join(get_base_dir(), 'source_environment.sh')
            env_dir = os.path.join(get_base_dir(), 'checkout', self.name)
            file.write("source {} {}".format(main_source, env_dir))


        click.echo('Changed active environment to < {} >!\n\
To be able to use source commands such as for example \
roslaunch and mca commands, type \'fzsource\''.format(self.name))


class EnvironmentChooser(click.MultiCommand):
    def get_current_evironments(self):
        checkout_folder = os.path.join(get_base_dir(), 'checkout')
        # TODO possibly check whether the directory contains actual workspace
        return [dir for dir in os.listdir(checkout_folder) if os.path.isdir(os.path.join(checkout_folder, dir))]

    def list_commands(self, ctx):
        return self.get_current_evironments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        # TODO improve logging for invalid environments
        if name in self.get_current_evironments():
            cmd = EnvironmentChoice(name=name, add_help_option=False)
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            return None


@cli.command('change_environment', cls=EnvironmentChooser, short_help='Add a new environment', invoke_without_command=True)
@click.pass_context
def change_environment(ctx):
    """Changes the global environment to the specified one. All environment-specific commands are then executed relative to that environment. \
       Only one environment can be active at a time."""
    if ctx.invoked_subcommand is None:
        click.echo('No environment specified. Please choose one of the available environments!')
    else:
        pass
    # print(env_name)


@cli.command()
@click.option('--count', default=1, help='Number of greetings.')
@click.option('--name', prompt='Your name', default="Nobody",
              help='The person to greet.')
def greet(count, name):
    """Simple program that greets NAME for a total of COUNT times."""
    for x in range(count):
        click.echo('Hello %s!' % name)
