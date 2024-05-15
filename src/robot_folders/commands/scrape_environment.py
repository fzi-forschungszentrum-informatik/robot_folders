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
"""Command that scrapes an environment's configuration into a config file"""
import os
import click

from yaml import safe_dump as yaml_safe_dump

from robot_folders.helpers.directory_helpers import (
    get_checkout_dir,
    get_catkin_dir,
    get_colcon_dir,
    list_environments,
)
from robot_folders.helpers.repository_helpers import create_rosinstall_entry


class EnvironmentScraper(click.Command):
    """Class that implements the command"""

    def __init__(self, name=None, **attrs):
        click.Command.__init__(self, name, **attrs)

        self.use_commit_id = False

    def invoke(self, ctx):
        env_dir = os.path.join(get_checkout_dir(), self.name)
        misc_ws_pkg_dir = os.path.join(env_dir, "misc_ws")
        catkin_dir = get_catkin_dir(env_dir)
        catkin_src_dir = os.path.join(catkin_dir, "src")
        colcon_dir = get_colcon_dir(env_dir)
        colcon_src_dir = os.path.join(colcon_dir, "src")
        demos_dir = os.path.join(env_dir, "demos")

        self.use_commit_id = ctx.parent.params["use_commit_id"]

        yaml_data = dict()

        if os.path.isdir(misc_ws_pkg_dir):
            click.echo("Scraping misc_ws_dir")
            yaml_data["misc_ws"] = dict()
            yaml_data["misc_ws"]["rosinstall"] = self.parse_folder(misc_ws_pkg_dir)

        if os.path.isdir(catkin_src_dir):
            click.echo("Scraping catkin workspace")
            yaml_data["catkin_workspace"] = dict()
            yaml_data["catkin_workspace"]["rosinstall"] = self.parse_folder(
                catkin_src_dir
            )

        if os.path.isdir(colcon_src_dir):
            click.echo("Scraping colcon workspace")
            yaml_data["colcon_workspace"] = dict()
            yaml_data["colcon_workspace"]["rosinstall"] = self.parse_folder(
                colcon_src_dir
            )

        if os.path.isdir(demos_dir):
            yaml_data["demos"] = dict()
            script_list = [
                script_file
                for script_file in os.listdir(demos_dir)
                if os.path.isfile(os.path.join(demos_dir, script_file))
                and os.access(os.path.join(demos_dir, script_file), os.X_OK)
            ]
            for script in script_list:
                script_path = os.path.join(demos_dir, script)
                with open(script_path, "r") as filecontent:
                    # content = f.read()
                    yaml_data["demos"][script] = filecontent.read()
                    filecontent.close()

        yaml_stream = open(ctx.params["out_file"], "w")
        yaml_safe_dump(
            yaml_data, stream=yaml_stream, encoding="utf-8", allow_unicode=True
        )

    def parse_folder(self, folder):
        """Recursively parse subfolders"""
        repos = list()
        files = os.walk(folder)
        for elem in files:
            try:
                subfolder = elem[0]
            except UnicodeDecodeError:
                click.echo("Unicode parsing error within folder {}".format(folder))
                continue
            subfolder_abs = os.path.join(folder, subfolder)
            git_dir = os.path.join(subfolder_abs, ".git")
            local_name = os.path.relpath(subfolder, folder)
            if os.path.isdir(git_dir):
                click.echo(local_name)
                entry = create_rosinstall_entry(
                    subfolder_abs, local_name, self.use_commit_id
                )
                repos.append(entry)
        return sorted(repos, key=lambda k: k["git"]["local-name"])


class EnvironmentChooser(click.MultiCommand):
    """Select the requested environment"""

    def list_commands(self, ctx):
        return list_environments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in list_environments():
            cmd = EnvironmentScraper(
                name=name, params=[click.Argument(param_decls=["out_file"])]
            )
            return cmd
        else:
            click.echo("No environment with name < %s > found." % name)
            return None


@click.command(
    "scrape_environment",
    cls=EnvironmentChooser,
    short_help="Scrape an environment config to a config file",
    invoke_without_command=True,
)
@click.option(
    "--use_commit_id",
    is_flag=True,
    default=False,
    help="If checked, the exact commit IDs get scraped instead of branch names.",
)
@click.pass_context
def cli(ctx, use_commit_id):
    """Scrapes an environment configuration into a config file,
       so that it can be given to somebody else. \
       This config file can then be used to initialize the environment
       in another robot_folders configuration.
       """
    if ctx.invoked_subcommand is None:
        click.echo(
            "No environment specified. Please choose one "
            "of the available environments!"
        )
