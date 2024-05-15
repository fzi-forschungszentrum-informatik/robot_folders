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
