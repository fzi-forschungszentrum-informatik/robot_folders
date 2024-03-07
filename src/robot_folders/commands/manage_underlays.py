"""This command populates the 'cd' functionality"""
import os
import click

from robot_folders.helpers.workspace_chooser import WorkspaceChooser
import robot_folders.helpers.directory_helpers as dir_helpers
from robot_folders.helpers.underlays import UnderlayManager


@click.command(
    "manage_underlays",
    short_help="Manage underlay workspaces used for the current workspace.",
)
def cli():
    """Opens a selection menu to manage the underlay environments used for the workspace."""

    current_env = dir_helpers.get_active_env()

    if current_env is None:
        click.echo(
            "Currently, there is no sourced environment. Please source one \
before calling the manage_underlays function."
        )
        return
    underlay_manager = UnderlayManager(current_env)
    underlays = underlay_manager.read_underlay_file()
    print(underlays)
    underlay_manager.query_underlays(active_list=underlays)
    underlay_manager.write_underlay_file()
