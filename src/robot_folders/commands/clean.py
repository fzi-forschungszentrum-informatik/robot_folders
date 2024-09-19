#
# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
"""Command to perform environment builds"""
import click

from robot_folders.helpers.workspace_chooser import WorkspaceChooser
import robot_folders.helpers.clean_helpers as clean
from robot_folders.helpers.directory_helpers import get_active_env


class CleanChooser(WorkspaceChooser):
    """Checks which workspaces are inside an env and returns these as subcommands"""

    def get_command(self, ctx, name):
        if get_active_env() is None:
            # click.echo("Currently, there is no sourced environment. "
            #            "Please source one before calling the make function.")
            return self

        if name in self.list_commands(ctx):
            if name == "ros":
                return clean.CatkinCleaner(name=name, add_help_option=False)
            elif name == "colcon":
                return clean.ColconCleaner(name=name, add_help_option=False)
            elif name == "misc":
                click.echo("Misc workspaces have to cleaned by hand. Doing nothing")
                return None
        else:
            click.echo("Did not find a workspace with the key < {} >.".format(name))
            return None

        return self


@click.command(
    "clean",
    cls=CleanChooser,
    invoke_without_command=True,
    short_help="Cleans an environment",
)
@click.pass_context
def cli(ctx):
    """ Cleans the currently active environment (deletes build and install files).
    You can choose to only clean one of \
the workspaces by adding the respective arg. Use tab completion to see which \
workspaces are present.
    """
    if get_active_env() is None:
        click.echo(
            "Currently, there is no sourced environment. Please source one \
before calling the clean function."
        )
        return

    if ctx.invoked_subcommand is None and ctx.parent.invoked_subcommand == "clean":
        click.echo("clean called without argument. Cleaning everything")

        # Check which workspaces are present
        cmd = CleanChooser(ctx)

        # Build all present workspaces individually
        for workspace in cmd.list_commands(ctx):
            clean_cmd = cmd.get_command(ctx, workspace)
            clean_cmd.invoke(ctx)
    return
