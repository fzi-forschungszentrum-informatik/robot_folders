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
import subprocess

from robot_folders.helpers.workspace_chooser import WorkspaceChooser
import robot_folders.helpers.build_helpers as build
from robot_folders.helpers.directory_helpers import get_active_env
from robot_folders.helpers.exceptions import ModuleException


class BuildChooser(WorkspaceChooser):
    """Checks which workspaces are inside an env and returns these as subcommands"""

    def get_command(self, ctx, name):
        if get_active_env() is None:
            # click.echo("Currently, there is no sourced environment. "
            #            "Please source one before calling the make function.")
            return self

        if name in self.list_commands(ctx):
            if name == "ros":
                return build.CatkinBuilder(name=name, add_help_option=False)
            elif name == "colcon":
                return build.ColconBuilder(name=name, add_help_option=True)
            elif name == "misc":
                click.echo("Misc workspaces have to built by hand. Doing nothing")
                return None
        else:
            click.echo("Did not find a workspace with the key < {} >.".format(name))
            return None

        return self

    def invoke(self, ctx):
        ### may raise an error.
        super(BuildChooser, self).invoke(ctx)


@click.command(
    "make",
    cls=BuildChooser,
    invoke_without_command=True,
    short_help="Builds an environment",
)
@click.pass_context
def cli(ctx):
    """ Builds the currently active environment. You can choose to only build one of \
the workspaces by adding the respective arg. Use tab completion to see which \
workspaces are present.
    """
    if get_active_env() is None:
        click.echo(
            "Currently, there is no sourced environment. Please source one \
before calling the make function."
        )
        return

    if ctx.invoked_subcommand is None and ctx.parent.invoked_subcommand == "make":
        click.echo("make called without argument. Building everything")

        # Check which workspaces are present
        cmd = BuildChooser(ctx)

        # Build all present workspaces individually
        for workspace in cmd.list_commands(ctx):
            build_cmd = cmd.get_command(ctx, workspace)
            build_cmd.invoke(ctx)
    return
