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
"""This command implements the change_environment functionality"""
import os
import click
import subprocess

import robot_folders.helpers.directory_helpers as dir_helpers
from robot_folders.helpers.exceptions import ModuleException


def set_active_env(env_name):
    with open(
        os.path.join(dir_helpers.get_checkout_dir(), ".cur_env"), "w"
    ) as cur_env_file:
        cur_env_file.write("{}".format(env_name))


class EnvironmentChoice(click.Command):
    """
    Writes the chosen environment into the cur_env temp file.
    NOTE: The actual sourcing takes place in a a shell script that reads that file's content
    """

    def invoke(self, ctx):
        set_active_env(self.name)


class EnvironmentChooser(click.MultiCommand):
    """Class that helps finding and choosing an environment."""

    def list_commands(self, ctx):
        return dir_helpers.list_environments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in dir_helpers.list_environments():
            cmd = EnvironmentChoice(name=name, add_help_option=False)
            return cmd
        else:
            click.echo("No environment with name < %s > found." % name)
            raise ModuleException("unknown environment", "EnvironmentChoice", 1)

    def invoke(self, ctx):
        try:
            super(EnvironmentChooser, self).invoke(ctx)
        except subprocess.CalledProcessError as err:
            raise (ModuleException(str(err), "change"))


@click.command(
    "change_environment",
    cls=EnvironmentChooser,
    short_help="Source an existing environment",
    invoke_without_command=True,
)
@click.pass_context
def cli(ctx):
    """
    Changes the global environment to the specified one. All environment-specific commands
    are then executed relative to that environment.
    Only one environment can be active at a time.
    """
    if ctx.invoked_subcommand is None:
        env_name = dir_helpers.get_last_activated_env()
        if env_name is not None:
            active_env_name = os.environ.get("ROB_FOLDERS_ACTIVE_ENV")
            if active_env_name:
                click.echo(f"Re-sourcing {active_env_name}")
                set_active_env(active_env_name)
            else:
                click.echo(
                    "No environment specified. Sourcing the most recent active environment: {}".format(
                        env_name
                    )
                )
    else:
        pass
    # print(env_name)
