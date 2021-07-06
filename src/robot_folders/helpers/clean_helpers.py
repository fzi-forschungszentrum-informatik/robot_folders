"""Module that helps cleaning workspaces"""
import os
import shutil
import click

from helpers.directory_helpers import get_active_env_path, mkdir_p, get_catkin_dir, get_colcon_dir
from helpers.which import which
from helpers import config_helpers

def clean_folder(folder):
    """Deletes everything inside a given folder. The folder itself is not deleted."""
    click.echo('Cleaning everything in {}'.format(folder))
    if os.path.isdir(folder):
        for the_file in os.listdir(folder):
            file_path = os.path.join(folder, the_file)
            if os.path.islink(file_path):
                click.echo('Deleting symlink {}'.format(file_path))
                os.unlink(file_path)
            elif os.path.isfile(file_path):
                click.echo('Deleting file {}'.format(file_path))
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                click.echo('Deleting folder {}'.format(file_path))
                shutil.rmtree(file_path)
    else:
        click.echo('Skipping non-existing folder "{}"'.format(folder))

def confirm_deletion(delete_list):
    """Requests a confirmation from the user that the mentioned paths should be deleted"""
    click.echo('Going to delete all files inside the following paths:\n{}'
               .format('\n'.join(delete_list)))

    confirm = click.prompt(
        "Please confirm by typing 'clean' (case sensitive).\nWARNING: "
        "After this all above mentioned paths will be cleaned and cannot be recovered! "
        "If you wish to abort your delete request, type 'abort'",
        type=click.Choice(['clean', 'abort']),
        default='abort')
    return confirm == 'clean'

class Cleaner(click.Command):
    """General cleaner class"""

    # Dummy variable to satisfy the linter
    clean_list = list()

    def clean(self):
        """General clean function"""
        if confirm_deletion(self.clean_list):
            for folder in self.clean_list:
                clean_folder(folder)
        else:
            click.echo('Cleaning not confirmed. Aborting now')
        click.echo('')


class IcCleaner(Cleaner):
    """Cleaner class for an ic workspace"""
    def invoke(self, ctx):
        click.echo('========== Cleaning ic workspace ==========')
        ic_dir = os.path.realpath(os.path.join(get_active_env_path(), 'ic_workspace'))
        self.clean_list.append(os.path.realpath(os.path.join(ic_dir, 'build')))
        self.clean_list.append(os.path.realpath(os.path.join(ic_dir, 'export')))
        self.clean()


class CatkinCleaner(Cleaner):
    """Cleaner class for catkin workspace"""
    def invoke(self, ctx):
        click.echo('========== Cleaning catkin workspace ==========')
        catkin_dir = get_catkin_dir()
        self.clean_list.append(os.path.join(catkin_dir, 'build'))
        self.clean_list.append(os.path.join(catkin_dir, 'build_isolated'))
        self.clean_list.append(os.path.join(catkin_dir, 'devel'))
        self.clean_list.append(os.path.join(catkin_dir, 'devel_isolated'))
        self.clean_list.append(os.path.join(catkin_dir, 'install'))
        self.clean_list.append(os.path.join(catkin_dir, 'install_isolated'))
        click.echo("Cleaning catkin_workspace in {}".format(catkin_dir))
        self.clean()

class ColconCleaner(Cleaner):
    """Cleaner class for colcon workspace"""
    def invoke(self, ctx):
        click.echo('========== Cleaning colcon workspace ==========')
        colcon_dir = get_colcon_dir()
        self.clean_list.append(os.path.join(colcon_dir, 'build'))
        self.clean_list.append(os.path.join(colcon_dir, 'log'))
        self.clean_list.append(os.path.join(colcon_dir, 'install'))
        click.echo("Cleaning colcon_workspace in {}".format(colcon_dir))
        self.clean()

class McaCleaner(Cleaner):
    """Cleaner class for an mca workspace"""
    def invoke(self, ctx):
        click.echo('========== Cleaning mca workspace ==========')
        mca_dir = os.path.realpath(os.path.join(get_active_env_path(), 'mca_workspace'))
        build_dir = os.path.realpath(os.path.join(mca_dir, 'build'))
        self.clean_list.append(build_dir)
        click.echo("Cleaning mca_workspace in {}".format(build_dir))
        self.clean()
