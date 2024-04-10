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
"""Module that helps cleaning workspaces"""
import os
import shutil
import click

from robot_folders.helpers.directory_helpers import (
    get_active_env_path,
    mkdir_p,
    get_catkin_dir,
    get_colcon_dir,
)
from robot_folders.helpers.which import which
from robot_folders.helpers import config_helpers


def clean_folder(folder):
    """Deletes everything inside a given folder. The folder itself is not deleted."""
    click.echo("Cleaning everything in {}".format(folder))
    if os.path.isdir(folder):
        for the_file in os.listdir(folder):
            file_path = os.path.join(folder, the_file)
            if os.path.islink(file_path):
                click.echo("Deleting symlink {}".format(file_path))
                os.unlink(file_path)
            elif os.path.isfile(file_path):
                click.echo("Deleting file {}".format(file_path))
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                click.echo("Deleting folder {}".format(file_path))
                shutil.rmtree(file_path)
    else:
        click.echo('Skipping non-existing folder "{}"'.format(folder))


def confirm_deletion(delete_list):
    """Requests a confirmation from the user that the mentioned paths should be deleted"""
    click.echo(
        "Going to delete all files inside the following paths:\n{}".format(
            "\n".join(delete_list)
        )
    )

    confirm = click.prompt(
        "Please confirm by typing 'clean' (case sensitive).\nWARNING: "
        "After this all above mentioned paths will be cleaned and cannot be recovered! "
        "If you wish to abort your delete request, type 'abort'",
        type=click.Choice(["clean", "abort"]),
        default="abort",
    )
    return confirm == "clean"


class Cleaner(click.Command):
    """General cleaner class"""

    # Dummy variable to satisfy the linter
    clean_list = list()

    def clean(self):
        """General clean function"""
        if confirm_deletion(self.clean_list):
            for folder in self.clean_list:
                clean_folder(folder)
        else:
            click.echo("Cleaning not confirmed. Aborting now")
        click.echo("")


class CatkinCleaner(Cleaner):
    """Cleaner class for catkin workspace"""

    def invoke(self, ctx):
        click.echo("========== Cleaning catkin workspace ==========")
        catkin_dir = get_catkin_dir()
        self.clean_list.append(os.path.join(catkin_dir, "build"))
        self.clean_list.append(os.path.join(catkin_dir, "build_isolated"))
        self.clean_list.append(os.path.join(catkin_dir, "devel"))
        self.clean_list.append(os.path.join(catkin_dir, "devel_isolated"))
        self.clean_list.append(os.path.join(catkin_dir, "install"))
        self.clean_list.append(os.path.join(catkin_dir, "install_isolated"))
        click.echo("Cleaning catkin_workspace in {}".format(catkin_dir))
        self.clean()


class ColconCleaner(Cleaner):
    """Cleaner class for colcon workspace"""

    def invoke(self, ctx):
        click.echo("========== Cleaning colcon workspace ==========")
        colcon_dir = get_colcon_dir()
        self.clean_list.append(os.path.join(colcon_dir, "build"))
        self.clean_list.append(os.path.join(colcon_dir, "log"))
        self.clean_list.append(os.path.join(colcon_dir, "install"))
        click.echo("Cleaning colcon_workspace in {}".format(colcon_dir))
        self.clean()
