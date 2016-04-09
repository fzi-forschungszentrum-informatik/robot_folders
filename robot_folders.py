import click
import os
import subprocess

@click.group()
def cli():
    """A simple command line tool."""

@cli.command('add_environment', short_help='Add a new environment')
@click.option('--no_ic', is_flag=True, default=False, help='Will not create an ic_workspace.')
@click.option('--no_catkin', is_flag=True, default=False, help='Will not create a catkin_workspace.')
@click.option('--no_mca2', is_flag=True, default=False, help='Will not create an mca2_workspace.')
@click.option('--use_ninja', is_flag=True, default=False,
              help='Use ninja as build system instead of linux makefiles.')
@click.option('--ros_distro', default="indigo", help='ROS distribution that should be used')
@click.argument('env_name')
#@click.option('-m', '--mca2_workspace', default=False, help='Create an mac2_workspace.')
def add_environment(no_ic, no_catkin, no_mca2, use_ninja, ros_distro, env_name):
    """Adds a new environment and creates the basic needed folders, e.g. a ic_orkspace and a catkin_workspace."""
    # TODO:
    #     - Add support for building in no_backup
    #     - Make building optional

    base_dir = os.path.dirname(os.path.realpath(__file__))
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
    if no_ic:
        click.echo("Requested to not create an IC Workspace")
    else:
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


    # Check if we should create a catkin workspace and create one if desired
    if no_catkin:
        click.echo("Requested to not create a catkin_workspace")
    else:
        click.echo("Creating catkin_workspace")
        os.mkdir(catkin_directory)
        os.mkdir(os.path.join(catkin_directory, "src"))

        cama_command = "source /opt/ros/{}/setup.bash && catkin_make {}".format(ros_distro, cama_flags)

        process = subprocess.Popen(["bash", "-c", cama_command],
                                   cwd=catkin_directory)
        process.wait()

    click.echo("Initial workspace setup completed")


    # Check if we should create an mca2 workspace and create one if desired
    if no_mca2:
        click.echo("Requested to not create an mca2 workspace")
    else:
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


@cli.command('delete', short_help='delete the repo')
def delete():
    """Deletes the repository."""







@cli.command()
@click.option('--count', default=1, help='Number of greetings.')
@click.option('--name', prompt='Your name', default="Nobody",
              help='The person to greet.')
def greet(count, name):
    """Simple program that greets NAME for a total of COUNT times."""
    for x in range(count):
        click.echo('Hello %s!' % name)
