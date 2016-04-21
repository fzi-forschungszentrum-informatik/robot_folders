import click
import os
from helpers.workspace_chooser import WorkspaceChooser
from helpers.directory_helpers import *

class CdChooser(WorkspaceChooser):
    def get_command(self, ctx, name):
        target_dir = get_active_env_path()
        if name in self.list_commands(ctx):
            if name == 'ic':
                target_dir = os.path.join(target_dir, 'ic_workspace')
            elif name == 'ros':
                target_dir = os.path.join(target_dir, 'catkin_workspace')
            elif name == 'mca':
                target_dir = os.path.join(target_dir, 'mca_workspace')
        else:
            click.echo('Did not find a workspace with the key < {} >.'.format(name))
            return None

        if get_active_env() == None:
            click.echo("No active environment found. Using most recently activated \
environment '{}'".format(get_last_activated_env()))
        click.echo("cd {}".format(target_dir))
        return self

@click.command(cls=CdChooser, invoke_without_command=True,
               short_help='CDs to a workspace inside the active environment')
@click.pass_context
def cli(ctx):
    """ CDs to a workspace inside the active environment
    """

    if ctx.invoked_subcommand is None and ctx.parent.invoked_subcommand == 'cd':
        if get_active_env() == None:
            click.echo("No active environment found. Using most recently activated \
environment '{}'".format(get_last_activated_env()))
        click.echo("cd {}".format(get_active_env_path()))
    return
