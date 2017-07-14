import click
import os

import helpers.directory_helpers as directory_helpers


class EnvironmentDeleter(click.Command):

    def invoke(self, ctx):
        env_dir = os.path.join(directory_helpers.get_checkout_dir(), self.name)
        catkin_dir = directory_helpers.get_catkin_dir(env_dir)
        ic_dir = os.path.join(env_dir, 'ic_workspace')
        mca_dir = os.path.join(env_dir, 'mca_workspace')

        delete_list = list()

        # Catkin workspace
        if catkin_dir is not None:
            catkin_build = os.path.join(catkin_dir, 'build')
            catkin_devel = os.path.join(catkin_dir, 'devel')
            catkin_install = os.path.join(catkin_dir, 'install')
            self.appendToListIfSymlink(catkin_build, delete_list)
            self.appendToListIfSymlink(catkin_devel, delete_list)
            self.appendToListIfSymlink(catkin_install, delete_list)
            self.appendToListIfFolder(catkin_dir, delete_list)
        else:
            click.echo('No catkin workspace found')

        # Ic-Workspace
        if os.path.exists(ic_dir):
            ic_build = os.path.join(ic_dir, 'build')
            ic_export = os.path.join(ic_dir, 'export')
            self.appendToListIfSymlink(ic_build, delete_list)
            self.appendToListIfSymlink(ic_export, delete_list)
            self.appendToListIfFolder(ic_dir, delete_list)
        else:
            click.echo('No Ic-Workspace found')

        # mca-Workspace
        if os.path.exists(mca_dir):
            mca_build = os.path.join(mca_dir, 'build')

            self.appendToListIfSymlink(mca_build, delete_list)
            self.appendToListIfFolder(mca_dir, delete_list)
        else:
            click.echo('No mca-Workspace found')

        delete_list.append(env_dir)

        click.echo('Going to delete the following paths:\n{}'.format('\n'.join(delete_list)))

        confirm = click.prompt(
            "Please confirm by typing in the environment name '{}' once again.\nWARNING:"
            "After this all environment files will be deleted and cannot be recovered! "
            "If you wish to abort your delete request, type 'abort'".format(
                self.name),
            type=click.Choice([self.name, 'abort']),
            default='abort')
        if confirm == self.name:
            click.echo('performing deletion!')
            for folder in delete_list:
                click.echo('Deleting {}'.format(folder))
                # TODO: Actually remove non-empty directory
        else:
            click.echo('Delete request aborted. Nothing happened.')

        click.echo('Successfully deleted environment \'{}\''.format(self.name))

    def appendToListIfSymlink(self, path, delete_list):
        """Checks whether the given path exists and is a symlink and appends it to the
        given list if both are true"""

        if os.path.exists(os.path.realpath(path)):
            if os.path.islink(path):
                delete_list.append(os.path.realpath(path))
                return True
        return False

    def appendToListIfFolder(self, path, delete_list):
        """Appends the given path to the given list, if it exists and is a folder.
        Returns true, if folder exists, false otherwise."""
        if os.path.exists(os.path.realpath(path)):
            if os.path.isdir(path):
                delete_list.append(os.path.realpath(path))
                return True
        return False


class EnvironmentChooser(click.MultiCommand):

    def get_current_evironments(self):
        checkout_folder = directory_helpers.get_checkout_dir()
        # TODO possibly check whether the directory contains actual workspace
        return [dir for dir in os.listdir(checkout_folder)
                if os.path.isdir(os.path.join(checkout_folder, dir))]

    def list_commands(self, ctx):
        return self.get_current_evironments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in self.get_current_evironments():
            cmd = EnvironmentDeleter(name=name)
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            return None


@click.command(cls=EnvironmentChooser,
               short_help='Deletes an environment from the checkout folder.',
               invoke_without_command=True)
@click.pass_context
def cli(ctx):
    """Removes an existing environment. This means that all files from this environments
    will be deleted from the checkout folder. If build or install directories are symlinked
    to another location (e.g. because it was build on no_backup), those will be deleted as well."""
    if ctx.invoked_subcommand is None:
        click.echo('No environment specified. Please choose one '
                   'of the available environments!')
