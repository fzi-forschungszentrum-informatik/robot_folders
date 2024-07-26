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
The workspace chooser is a click Multicommand to generate a command out of existing
workspaces inside an environment.
"""

import os
import click
from robot_folders.helpers.directory_helpers import (
    get_checkout_dir,
    get_active_env_path,
    get_catkin_dir,
    get_colcon_dir,
    get_misc_dir,
)


class WorkspaceChooser(click.MultiCommand):
    """
    The workspace chooser finds all existing environments.
    """

    def get_workspaces(self):
        """Searches all environments inside the checkout folder"""
        checkout_folder = get_checkout_dir()
        # TODO possibly check whether the directory contains actual workspace
        return [
            folder
            for folder in os.listdir(checkout_folder)
            if os.path.isdir(os.path.join(checkout_folder, folder))
        ]

    def list_commands(self, ctx):
        if get_active_env_path() is None:
            return list()
        workspaces = [folder for folder in os.listdir(get_active_env_path())]
        cmds = list()
        if os.path.exists(get_colcon_dir()):
            cmds.append("colcon")
        if os.path.exists(get_catkin_dir()):
            cmds.append("ros")
        if os.path.exists(get_misc_dir()):
            cmds.append("misc")

        return cmds

    def format_commands(self, ctx, formatter):
        return "ic, ros"
