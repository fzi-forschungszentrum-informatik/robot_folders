import click
import os
import subprocess

def get_base_dir():
    return os.path.dirname(os.path.realpath(__file__))

@click.group()
def cli():
    """A simple command line tool."""

@cli.command('add_environment', short_help='Add a new environment')
@click.option('--create_ic', is_flag=True, default=True,
              prompt="Would you like to create an ic_workspace?",
              help='Create an ic_workspace?')
@click.option('--create_catkin', is_flag=True, default=True,
              prompt="Would you like to create a catkin_workspace?",
              help='Will not create a catkin_workspace.')
@click.option('--create_mca2', is_flag=True, default=False,
              prompt="Would you like to create an mca2_workspace?",
              help='Will not create an mca2_workspace.')

@click.option('--use_ninja', is_flag=True, default=False,
              help='Use ninja as build system instead of linux makefiles.')
@click.argument('env_name')
#@click.option('-m', '--mca2_workspace', default=False, help='Create an mac2_workspace.')
def add_environment(create_ic, create_catkin, create_mca2, use_ninja, env_name):
    """Adds a new environment and creates the basic needed folders, e.g. a ic_orkspace and a catkin_workspace."""
    # TODO:
    #     - Add support for building in no_backup
    #     - Make building optional

    base_dir = get_base_dir()
    ic_repo_url = "git://idsgit.fzi.de/core/ic_workspace.git"
    ic_cmake_flags = ""
    cama_flags = ""
    mca_repo_url = "git://idsgit.fzi.de/mca2/mca2.git"
    mca_cmake_flags = ""


    if use_ninja:
        sep = " "
        ic_cmake_flags = sep.join([ic_cmake_flags, "-GNinja"])
        cama_flags = sep.join([cama_flags, "--use-ninja"])
        mca_cmake_flags = sep.join([mca_cmake_flags, "-GNinja"])


    if os.path.exists("{}/checkout/{}".format(base_dir, env_name)):
        click.echo("An environment with the name \"{}\" already exists. Exiting now.".format(env_name))
        return

    click.echo("Creating environment with name \"{}\"".format(env_name))
    os.mkdir("{}/checkout/{}".format(base_dir, env_name))
    ic_directory = os.path.join(base_dir, "checkout", env_name, "ic_workspace")
    catkin_directory = os.path.join(base_dir, "checkout", env_name, "catkin_workspace")
    mca_directory = os.path.join(base_dir, "checkout", env_name, "mca2_workspace")


    # Check if we should create an ic workspace and create one if desired
    if create_ic:
        click.echo("Creating ic_workspace")

        try:
            subprocess.check_call(["git", "clone", ic_repo_url, ic_directory])

            process = subprocess.Popen(["./IcWorkspace.py", "grab", "base"],
                                       cwd=ic_directory)
            process.wait()

            ic_build_dir = os.path.join(ic_directory, "build")
            os.mkdir(ic_build_dir)
            ic_cmake_cmd = "cmake .. {}".format(ic_cmake_flags)
            process = subprocess.Popen(["bash", "-c", ic_cmake_cmd],
                                       cwd=ic_build_dir)
            process.wait()

        except subprocess.CalledProcessError as e:
            click.echo(e.output)
            click.echo("An error occurred while creating the ic_workspace. Exiting now")
            return

    else:
        click.echo("Requested to not create an IC Workspace")


    # Check if we should create a catkin workspace and create one if desired
    if create_catkin:
        click.echo("Creating catkin_workspace")
        installed_ros_distros = os.listdir("/opt/ros")
        click.echo("Available ROS distributions: {}".format(installed_ros_distros))
        ros_distro = installed_ros_distros[0]
        if len(installed_ros_distros) > 1:
            ros_distro = click.prompt('Which ROS distribution would you like to use?',
                                      type=click.Choice(installed_ros_distros), default=installed_ros_distros[0])
        click.echo("Using ROS distribution \'{}\'".format(ros_distro))
        ros_global_dir = "/opt/ros/{}".format(ros_distro)
        copy_cmake_lists = click.prompt(("Would you like to copy the top-level CMakeLists.txt to the catkin"
                                        " src directory instead of using a symlink?"
                                        " (This is incredibly useful when using the QtCreator.)"),
                                        type=bool,
                                        default=True)


        os.mkdir(catkin_directory)
        os.mkdir(os.path.join(catkin_directory, "src"))

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




    # Check if we should create an mca2 workspace and create one if desired
    if create_mca2:
        click.echo("Creating mca2_workspace")

        try:
            subprocess.check_call(["git", "clone", mca_repo_url, mca_directory])

            process = subprocess.Popen(["script/git_clone_base.py"],
                                       cwd=mca_directory)
            process.wait()

            process = subprocess.Popen(["script/ic/update_cmake.py"],
                                       cwd=mca_directory)
            process.wait()

            mca_build_dir = os.path.join(mca_directory, "build")
            os.mkdir(mca_build_dir)
            mca_cmake_cmd = "cmake .. {}".format(mca_cmake_flags)
            process = subprocess.Popen(["bash", "-c", mca_cmake_cmd],
                                       cwd=mca_build_dir)
            process.wait()

        except subprocess.CalledProcessError as e:
            click.echo(e.output)
            click.echo("An error occurred while creating the ic_workspace. Exiting now")
            return
    else:
        click.echo("Requested to not create an mca2 workspace")


        click.echo("Initial workspace setup completed")





@cli.command('delete', short_help='delete the repo')
def delete():
    """Deletes the repository."""

class EnvironmentChoice(click.Command):
    def invoke(self, ctx):
        click.echo('Changed active environment to < %s >!' % self.name)
        pass

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
