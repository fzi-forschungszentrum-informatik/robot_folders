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
"""This command populates the 'cd' functionality"""
import os
import click

from robot_folders.helpers.workspace_chooser import WorkspaceChooser
import robot_folders.helpers.directory_helpers as dir_helpers


class CdCommand(click.Command):
    """Command to output a cd command"""

    def __init__(self, name=None, target_dir=None, **attrs):
        click.Command.__init__(self, name, **attrs)

        self.short_help = target_dir
        self.target_dir = target_dir

    def invoke(self, ctx):
        """Prints a cd command to the output"""
        click.echo("cd {}".format(self.target_dir))


class CdChooser(WorkspaceChooser):
    """Class implementing the cd command"""

    def get_command(self, ctx, name):
        env = dir_helpers.get_active_env()
        if env is None:
            click.echo(
                "No active environment found. Using most recently activated \
environment '{}'".format(
                    dir_helpers.get_last_activated_env()
                )
            )
            env = dir_helpers.get_last_activated_env()

        target_dir = dir_helpers.get_active_env_path()
        if name in self.list_commands(ctx):
            if name == "ros":
                target_dir = dir_helpers.get_catkin_dir()
            elif name == "colcon":
                target_dir = dir_helpers.get_colcon_dir()
            elif name == "misc":
                target_dir = dir_helpers.get_misc_dir()
        else:
            click.echo(
                "Did not find a workspace with the key < {} > inside "
                "current environment < {} >.".format(name, env)
            )
            return self

        return CdCommand(name=name, target_dir=target_dir)


@click.command(
    "cd",
    cls=CdChooser,
    invoke_without_command=True,
    short_help="CDs to a workspace inside the active environment",
)
@click.pass_context
def cli(ctx):
    """CDs to a workspace inside the active environment"""

    if ctx.invoked_subcommand is None and ctx.parent.invoked_subcommand == "cd":
        if dir_helpers.get_active_env() is None:
            click.echo(
                "No active environment found. Using most recently "
                "activated environment"
            )
        active_env_path = dir_helpers.get_active_env_path()
        if active_env_path is not None:
            click.echo("cd {}".format(active_env_path))
    return
