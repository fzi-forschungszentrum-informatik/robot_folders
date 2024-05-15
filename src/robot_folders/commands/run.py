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
"""Command to run scripts from the demos folder"""
import os
import subprocess
import click

from robot_folders.helpers.directory_helpers import get_active_env_path, get_active_env


def get_demo_binaries():
    """List all executable scripts in the demos folder"""
    demo_dir = os.path.join(get_active_env_path(), "demos")
    if not os.path.exists(demo_dir):
        return list()

    script_list = [
        script_file
        for script_file in os.listdir(demo_dir)
        if os.path.isfile(os.path.join(demo_dir, script_file))
        and os.access(os.path.join(demo_dir, script_file), os.X_OK)
    ]
    return script_list


class ScriptExecutor(click.Command):
    """Command implementation for script running"""

    def invoke(self, ctx):
        demo_dir = os.path.join(get_active_env_path(), "demos")
        process = subprocess.Popen(["bash", "-c", os.path.join(demo_dir, self.name)])
        process.wait()


class ScriptSelector(click.MultiCommand):
    """Class that helps finding the right demo script"""

    def list_commands(self, ctx):
        return get_demo_binaries()

    def get_command(self, ctx, name):
        if name in get_demo_binaries():
            cmd = ScriptExecutor(name=name)
            return cmd
        else:
            click.echo("No such executable: {}".format(name))
            return None


@click.command(
    "run",
    cls=ScriptSelector,
    short_help="Run a demo script",
    invoke_without_command=True,
)
@click.pass_context
def cli(ctx):
    """Runs an executable script inside the environment's 'demos' directory."""
    if get_active_env() is None:
        click.echo(
            "Currently, there is no sourced environment. "
            "Please source one before calling the make function."
        )
        return

    if ctx.invoked_subcommand is None:
        cmd = ScriptSelector(ctx)
        click.echo(
            "No demo script specified. Please specify one of {}".format(
                cmd.list_commands(ctx)
            )
        )
