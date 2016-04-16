import click
import os
from helpers.directory_helpers import get_base_dir, get_active_env_path


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


@click.command(cls=WorkspaceChooser, invoke_without_command=True,
               short_help='Builds an environment')
@click.pass_context
def cli(ctx):
    """ Builds the currently active environment. You can choose to only build one of \
the workspaces by adding the respective arg. Use tab completion to see which \
workspaces are present.
    """
    click.echo('call!')
    click.echo(ctx.params)
    click.echo(ctx.invoked_subcommand)
    exit(0)
    if ctx.invoked_subcommand is None:
        click.echo("make called without argument")
