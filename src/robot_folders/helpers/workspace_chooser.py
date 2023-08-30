"""
The workspace chooser is a click Multicommand to generate a command out of existing
workspaces inside an environment.
"""

import os
import click
from robot_folders.helpers.directory_helpers import (
    get_base_dir,
    get_active_env_path,
    get_catkin_dir,
    get_colcon_dir)

class WorkspaceChooser(click.MultiCommand):
    """
    The workspace chooser finds all existing environments.
    """

    def get_workspaces(self):
        """Searches all environments inside the checkout folder"""
        checkout_folder = os.path.join(get_base_dir(), 'checkout')
        # TODO possibly check whether the directory contains actual workspace
        return [folder for folder in os.listdir(checkout_folder) if
                os.path.isdir(os.path.join(checkout_folder, folder))]

    def list_commands(self, ctx):
        if get_active_env_path() is None:
            return list()
        workspaces = [folder for folder in os.listdir(get_active_env_path())]
        cmds = list()
        if os.path.exists(get_colcon_dir()):
            cmds.append('colcon')
        if os.path.exists(get_catkin_dir()):
            cmds.append('ros')

        return cmds

    def format_commands(self, ctx, formatter):
        return 'ic, ros'
