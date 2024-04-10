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
"""
Implements the delete command
"""
import os
import shutil
import getpass
import click

import robot_folders.helpers.directory_helpers as directory_helpers


def append_to_list_if_symlink(path, delete_list):
    """Checks whether the given path exists and is a symlink and appends it to the
    given list if both are true"""
    if os.path.exists(os.path.realpath(path)):
        if os.path.islink(path):
            delete_list.append(os.path.realpath(path))
            return True
    return False


def append_to_list_if_folder(path, delete_list):
    """Appends the given path to the given list, if it exists, is a folder
    and is not in the list yet.
    Returns true, if folder exists, false otherwise."""
    if os.path.exists(os.path.realpath(path)):
        if os.path.isdir(path) and os.path.realpath(path) not in delete_list:
            delete_list.append(os.path.realpath(path))
            return True
    return False


def delete_folder(path):
    """Deletes the given path, if it exists and is a folder.
    Returns true, if folder exists and was successfully delete."""
    if os.path.exists(path):
        if os.path.isdir(path):
            shutil.rmtree(path)
            return True
    return False


class EnvironmentDeleter(click.Command):
    """Command that deletes an environment"""

    def __init__(self, name=None, **attrs):
        click.Command.__init__(self, name, **attrs)

        self.force = False

    def invoke(self, ctx):
        env_dir = os.path.join(directory_helpers.get_checkout_dir(), self.name)
        catkin_dir = directory_helpers.get_catkin_dir(env_dir)

        delete_list = list()

        self.force = ctx.parent.params["force"]

        # Catkin workspace
        if catkin_dir is not None:
            catkin_build = os.path.join(catkin_dir, "build")
            catkin_devel = os.path.join(catkin_dir, "devel")
            catkin_install = os.path.join(catkin_dir, "install")
            append_to_list_if_symlink(catkin_build, delete_list)
            append_to_list_if_symlink(catkin_devel, delete_list)
            append_to_list_if_symlink(catkin_install, delete_list)
            append_to_list_if_folder(catkin_dir, delete_list)
        else:
            click.echo("No catkin workspace found")

        delete_list.append(env_dir)

        # no_backup build base
        build_base_dir = directory_helpers.get_build_base_dir(use_no_backup=True)
        append_to_list_if_folder(os.path.join(build_base_dir, self.name), delete_list)

        click.echo(
            "Going to delete the following paths:\n{}".format("\n".join(delete_list))
        )
        confirmed = False
        if self.force:
            confirmed = True
        else:
            confirm = click.prompt(
                "Please confirm by typing in the environment name '{}' once again.\nWARNING: "
                "After this all environment files will be deleted and cannot be recovered! "
                "If you wish to abort your delete request, type 'abort'".format(
                    self.name
                ),
                type=click.Choice([self.name, "abort"]),
                default="abort",
            )
            if confirm == self.name:
                confirmed = True

        if confirmed:
            click.echo("performing deletion!")
            for folder in delete_list:
                click.echo("Deleting {}".format(folder))
                delete_folder(folder)
            click.echo("Successfully deleted environment '{}'".format(self.name))
        else:
            click.echo("Delete request aborted. Nothing happened.")


class EnvironmentChooser(click.MultiCommand):
    """Choose an environment"""

    def list_commands(self, ctx):
        return directory_helpers.list_environments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in directory_helpers.list_environments():
            cmd = EnvironmentDeleter(name=name)
            return cmd
        else:
            click.echo("No environment with name < %s > found." % name)
            return None


@click.command(
    "delete_environment",
    cls=EnvironmentChooser,
    short_help="Deletes an environment from the checkout folder.",
    invoke_without_command=True,
)
@click.option(
    "--force",
    default=False,
    is_flag=True,
    help="Skip confirmation and delete directly. This is meant for automated runs only.",
)
@click.pass_context
def cli(ctx, force):
    """Removes an existing environment. This means that all files from this environment
    will be deleted from the checkout folder. If build or install directories are symlinked
    to another location (e.g. because it was built on no_backup), those will be deleted as well.
    """
    if ctx.invoked_subcommand is None:
        click.echo(
            "No environment specified. Please choose one "
            "of the available environments!"
        )
