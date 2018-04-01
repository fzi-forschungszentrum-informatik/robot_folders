"""Command to perform environment builds"""
import click

from helpers.workspace_chooser import WorkspaceChooser
import helpers.clean_helpers as clean
from helpers.directory_helpers import get_active_env


class CleanChooser(WorkspaceChooser):
    """Checks which workspaces are inside an env and returns these as subcommands"""

    def get_command(self, ctx, name):
        if get_active_env() is None:
            # click.echo("Currently, there is no sourced environment. "
            #            "Please source one before calling the make function.")
            return self

        if name in self.list_commands(ctx):
            if name == 'ic':
                click.echo('========== Cleaning ic workspace ==========')
                return clean.IcCleaner(name=name, add_help_option=False)
            elif name == 'ros':
                click.echo('========== Cleaning ros workspace ==========')
                return clean.CatkinCleaner(name=name, add_help_option=False)
            elif name == 'mca':
                click.echo('========== Cleaning mca workspace ==========')
                return clean.McaCleaner(name=name, add_help_option=False)
        else:
            click.echo('Did not find a workspace with the key < {} >.'.format(name))
            return None

        return self


@click.command(cls=CleanChooser, invoke_without_command=True,
               short_help='Cleans an environment')
@click.pass_context
def cli(ctx):
    """ Cleans the currently active environment (deletes build and install files).
    You can choose to only clean one of \
the workspaces by adding the respective arg. Use tab completion to see which \
workspaces are present.
    """
    if get_active_env() is None:
        click.echo("Currently, there is no sourced environment. Please source one \
before calling the clean function.")
        return

    if ctx.invoked_subcommand is None and ctx.parent.invoked_subcommand == 'clean':
        click.echo("clean called without argument. Cleaning everything")

        # Check which workspaces are present
        cmd = CleanChooser(ctx)

        # Build all present workspaces individually
        for workspace in cmd.list_commands(ctx):
            clean_cmd = cmd.get_command(ctx, workspace)
            clean_cmd.invoke(ctx)
    return
