import click
import os
from helpers.workspace_chooser import WorkspaceChooser
import helpers.build_helpers as build
from helpers.directory_helpers import get_active_env

class BuildChooser(WorkspaceChooser):
    def get_command(self, ctx, name):
        if name in self.list_commands(ctx):
            if name == 'ic':
                return build.IcBuilder(name=name, add_help_option=False)
            elif name == 'ros':
                return build.CatkinBuilder(name=name, add_help_option=False)
            elif name == 'mca':
                return build.McaBuilder(name=name, add_help_option=False)
        else:
            click.echo('Did not find a workspace with the key < {} >.'.format(name))
            return None

        return self


@click.command(cls=BuildChooser, invoke_without_command=True,
               short_help='Builds an environment')
@click.pass_context
def cli(ctx):
    """ Builds the currently active environment. You can choose to only build one of \
the workspaces by adding the respective arg. Use tab completion to see which \
workspaces are present.
    """

    if get_active_env() == None:
        click.echo("Currently, there is no sourced environment. Please source one \
before calling the make function.")
        return

    if ctx.invoked_subcommand is None and ctx.parent.invoked_subcommand == 'make':
        click.echo("make called without argument. Building everything")

        # Check which workspaces are present
        cmd = BuildChooser(ctx)

        # Build all present workspaces individually
        for ws in cmd.list_commands(ctx):
            build_cmd = cmd.get_command(ctx, ws)
            build_cmd.invoke(ctx)
    return
