"""Command to perform environment builds"""
import click
import subprocess

from helpers.workspace_chooser import WorkspaceChooser
import helpers.build_helpers as build
from helpers.directory_helpers import get_active_env
from helpers.exceptions import ModuleException


class BuildChooser(WorkspaceChooser):
    """Checks which workspaces are inside an env and returns these as subcommands"""

    def get_command(self, ctx, name):
        if get_active_env() is None:
            # click.echo("Currently, there is no sourced environment. "
            #            "Please source one before calling the make function.")
            return self

        if name in self.list_commands(ctx):
            if name == 'ros':
                return build.CatkinBuilder(name=name, add_help_option=False)
            elif name == 'colcon':
                return build.ColconBuilder(name=name, add_help_option=False)
        else:
            click.echo('Did not find a workspace with the key < {} >.'.format(name))
            return None

        return self

    def invoke(self, ctx):
        ### may raise an error.
        super(BuildChooser, self).invoke(ctx)

@click.command('make', cls=BuildChooser, invoke_without_command=True,
               short_help='Builds an environment')
@click.pass_context
def cli(ctx):
    """ Builds the currently active environment. You can choose to only build one of \
the workspaces by adding the respective arg. Use tab completion to see which \
workspaces are present.
    """
    if get_active_env() is None:
        click.echo("Currently, there is no sourced environment. Please source one \
before calling the make function.")
        return

    if ctx.invoked_subcommand is None and ctx.parent.invoked_subcommand == 'make':
        click.echo("make called without argument. Building everything")

        # Check which workspaces are present
        cmd = BuildChooser(ctx)

        # Build all present workspaces individually
        for workspace in cmd.list_commands(ctx):
            build_cmd = cmd.get_command(ctx, workspace)
            build_cmd.invoke(ctx)
    return 

