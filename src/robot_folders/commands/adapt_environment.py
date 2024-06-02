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
This implements a command to adapt an environment with a config file.
"""
from __future__ import print_function
import os
import stat
import subprocess
import click

import robot_folders.helpers.directory_helpers as dir_helpers
from robot_folders.helpers.repository_helpers import create_rosinstall_entry
from robot_folders.helpers.ConfigParser import ConfigFileParser
import robot_folders.helpers.environment_helpers as environment_helpers


class EnvironmentAdapter(click.Command):
    """
    Implements a click command interface to adapt an environment.
    """

    def __init__(self, name=None, **attrs):
        click.Command.__init__(self, name, **attrs)

        self.local_delete_policy = "ask"
        self.local_override_policy = "ask"
        self.ignore_catkin = False
        self.ignore_colcon = False
        self.ignore_misc = False
        self.no_submodules = False
        self.rosinstall = dict()

    def invoke(self, ctx):
        """
        This invokes the actual command.
        """
        env_dir = os.path.join(dir_helpers.get_checkout_dir(), self.name)
        catkin_dir = dir_helpers.get_catkin_dir(env_dir)
        colcon_dir = dir_helpers.get_colcon_dir(env_dir)
        catkin_src_dir = os.path.join(catkin_dir, "src")
        colcon_src_dir = os.path.join(colcon_dir, "src")
        misc_ws_dir = os.path.join(env_dir, "misc_ws")
        demos_dir = os.path.join(env_dir, "demos")

        self.local_delete_policy = ctx.parent.params["local_delete_policy"]
        self.local_override_policy = ctx.parent.params["local_override_policy"]
        self.ignore_catkin = ctx.parent.params["ignore_catkin"]
        self.ignore_colcon = ctx.parent.params["ignore_colcon"]
        self.ignore_misc = ctx.parent.params["ignore_misc"]
        self.no_submodules = ctx.parent.params["no_submodules"]

        config_file_parser = ConfigFileParser(ctx.params["in_file"])
        has_catkin, ros_rosinstall = config_file_parser.parse_ros_config()
        has_colcon, ros2_rosinstall = config_file_parser.parse_ros2_config()
        has_misc_ws, misc_ws_rosinstall = config_file_parser.parse_misc_ws_config()
        os.environ["ROB_FOLDERS_ACTIVE_ENV"] = self.name

        if has_misc_ws and (not self.ignore_misc):
            if os.path.isdir(misc_ws_dir):
                click.echo("Adapting misc workspace")
                self.rosinstall = dict()
                self.parse_folder(misc_ws_dir)
                if misc_ws_rosinstall:
                    self.adapt_rosinstall(misc_ws_rosinstall, misc_ws_dir)
            else:
                click.echo("Creating misc workspace")
                has_nobackup = dir_helpers.check_build_on_nobackup()
                build_base_dir = dir_helpers.get_build_base_dir(has_nobackup)
                misc_ws_build_root = os.path.join(build_base_dir, self.name, "misc_ws")

                environment_helpers.MiscCreator(
                    misc_ws_dir,
                    rosinstall=misc_ws_rosinstall,
                    build_root=misc_ws_build_root,
                    no_submodules=self.no_submodules,
                )

        if has_catkin and (not self.ignore_catkin):
            if os.path.isdir(catkin_src_dir):
                click.echo("Adapting catkin workspace")
                self.rosinstall = dict()
                self.parse_folder(catkin_src_dir)
                if ros_rosinstall:
                    self.adapt_rosinstall(ros_rosinstall, catkin_src_dir)
            else:
                click.echo("Creating catkin workspace")
                has_nobackup = dir_helpers.check_build_on_nobackup()
                catkin_dir = os.path.join(env_dir, "catkin_ws")
                build_base_dir = dir_helpers.get_build_base_dir(has_nobackup)
                catkin_build_dir = os.path.join(
                    build_base_dir, self.name, "catkin_ws", "build"
                )

                catkin_creator = environment_helpers.CatkinCreator(
                    catkin_directory=catkin_dir,
                    build_directory=catkin_build_dir,
                    rosinstall=ros_rosinstall,
                    no_submodules=self.no_submodules,
                )
                catkin_creator.create()

        if has_colcon and (not self.ignore_colcon):
            if os.path.isdir(colcon_src_dir):
                click.echo("Adapting colcon workspace")
                self.rosinstall = dict()
                self.parse_folder(colcon_src_dir)
                if ros2_rosinstall:
                    self.adapt_rosinstall(ros2_rosinstall, colcon_src_dir)
            else:
                click.echo("Creating colcon workspace")
                has_nobackup = dir_helpers.check_build_on_nobackup()
                colcon_dir = os.path.join(env_dir, "colcon_ws")
                build_base_dir = dir_helpers.get_build_base_dir(has_nobackup)
                colcon_build_dir = os.path.join(
                    build_base_dir, self.name, "colcon_ws", "build"
                )

                colcon_creator = environment_helpers.ColconCreator(
                    colcon_directory=colcon_dir,
                    build_directory=colcon_build_dir,
                    rosinstall=ros2_rosinstall,
                    no_submodules=self.no_submodules,
                )
                colcon_creator.create()

        click.echo("Looking for demo scripts")
        dir_helpers.mkdir_p(demos_dir)
        scripts = config_file_parser.parse_demo_scripts()
        for script in scripts:
            click.echo(
                "Found {} in config. Will be overwritten if file exists".format(script)
            )
            script_path = os.path.join(demos_dir, script)
            with open(script_path, mode="w") as demo_script:
                demo_script.write(scripts[script])
                demo_script.close()
            os.chmod(
                script_path,
                stat.S_IRWXU
                | stat.S_IRGRP
                | stat.S_IXGRP
                | stat.S_IROTH
                | stat.S_IXOTH,
            )

    def adapt_rosinstall(self, config_rosinstall, packages_dir, workspace_dir=""):
        """
        Parses the given config rosinstall and compares it to the locally installed packages
        """
        for repo in config_rosinstall:
            local_version_exists = False
            version_update_required = True
            uri_update_required = True
            local_name = ""
            uri = ""
            version = ""

            if "local-name" in repo["git"]:
                local_name = repo["git"]["local-name"]
            else:
                click.echo(
                    "No local-name given for package '{}'. "
                    "Skipping package".format(repo["git"])
                )
                continue
            package_dir = os.path.join(packages_dir, local_name)

            if "version" in repo["git"]:
                version = repo["git"]["version"]
            else:
                click.echo(
                    "WARNING: No version tag given for package '{}'. "
                    "The local version will be kept or the master "
                    "version will be checked out for new package".format(local_name)
                )

            if "uri" in repo["git"]:
                uri = repo["git"]["uri"]
            else:
                click.echo(
                    "WARNING: No uri given for package '{}'. "
                    "Skipping package".format(local_name)
                )
                continue

            # compare the repos' versions and uris
            if local_name in self.rosinstall.keys():
                local_version_exists = True
                local_repo = self.rosinstall[local_name]
                if version == "" or version == local_repo["git"]["version"]:
                    version_update_required = False
                elif self.local_override_policy == "keep_local":
                    version_update_required = False
                elif self.local_override_policy == "override":
                    version_update_required = True
                else:
                    click.echo(
                        "Package '{}' version differs from local version. ".format(
                            local_name
                        )
                    )
                    click.echo(
                        "1) local version: {}".format(local_repo["git"]["version"])
                    )
                    click.echo("2) config_file version: {}".format(version))
                    version_to_keep = click.prompt(
                        "Which version should be used?",
                        type=click.Choice(["1", "2"]),
                        default="1",
                    )
                    version_update_required = version_to_keep == "2"

                if uri == local_repo["git"]["uri"]:
                    uri_update_required = False
                elif self.local_override_policy == "keep_local":
                    uri_update_required = False
                elif self.local_override_policy == "override":
                    uri_update_required = True
                else:
                    click.echo(
                        "Package '{}' uri differs from local version. ".format(
                            local_name
                        )
                    )
                    click.echo("local version: {}".format(local_repo["git"]["uri"]))
                    click.echo("config_file version: {}".format(uri))
                    version_to_keep = click.prompt(
                        "Which uri should be used?",
                        type=click.Choice(["1", "2"]),
                        default="2",
                    )
                    uri_update_required = version_to_keep == "2"

            else:
                click.echo(
                    "Package '{}' does not exist in local structure. "
                    "Going to download.".format(local_name)
                )

            # Create repo if it does not exist yet.
            if not local_version_exists:
                if self.no_submodules:
                    subprocess.check_call(["git", "clone", uri, package_dir])
                else:
                    subprocess.check_call(
                        ["git", "clone", uri, package_dir, "--recurse-submodules"]
                    )

            # Change the origin to the uri specified
            if uri_update_required:
                process = subprocess.check_call(
                    ["git", "remote", "set-url", "origin", uri], cwd=package_dir
                )

            # Checkout the version specified
            if version_update_required:
                process = subprocess.check_call(["git", "fetch"], cwd=package_dir)
                process = subprocess.check_call(
                    ["git", "checkout", version], cwd=package_dir
                )

        config_name_list = [d["git"]["local-name"] for d in config_rosinstall]

        for repo in self.rosinstall:
            if repo not in config_name_list:
                click.echo(
                    "Package '{}' found locally, but not in config.".format(repo)
                )
                local_name = self.rosinstall[repo]["git"]["local-name"]
                package_dir = os.path.join(packages_dir, local_name)
                if self.local_delete_policy == "delete_all":
                    dir_helpers.recursive_rmdir(package_dir)
                    click.echo("Deleted '{}'".format(repo))
                elif self.local_delete_policy == "ask":
                    if click.confirm("Do you want to delete it?"):
                        dir_helpers.recursive_rmdir(package_dir)
                        click.echo("Deleted '{}'".format(repo))
                elif self.local_delete_policy == "keep_all":
                    click.echo("Keeping repository as all should be kept")

    def parse_folder(self, folder, prefix=""):
        """
        Function to recursively find git repositories
        """
        subfolders = os.listdir(folder)
        for subfolder in subfolders:
            subfolder_abs = os.path.join(folder, subfolder)
            if os.path.isdir(subfolder_abs):
                git_dir = os.path.join(subfolder_abs, ".git")
                local_name = os.path.join(prefix, subfolder)
                if os.path.isdir(git_dir):
                    click.echo(
                        "Found '{}' in local folder structure.".format(local_name)
                    )
                    entry = create_rosinstall_entry(subfolder_abs, local_name)
                    self.rosinstall[local_name] = entry
                self.parse_folder(subfolder_abs, local_name)


class EnvironmentChooser(click.MultiCommand):
    """Class to select an environment"""

    def list_commands(self, ctx):
        return dir_helpers.list_environments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in dir_helpers.list_environments():
            cmd = EnvironmentAdapter(
                name=name, params=[click.Argument(param_decls=["in_file"])]
            )
            return cmd
        else:
            click.echo("No environment with name < %s > found." % name)
            return None


@click.command(
    "adapt_environment",
    cls=EnvironmentChooser,
    short_help="Adapt an environment to a config file",
    invoke_without_command=True,
)
@click.option(
    "--local_delete_policy",
    type=click.Choice(["delete_all", "keep_all", "ask"]),
    default="ask",
    help=(
        "Defines whether repositories existing local, but NOT "
        "in the config file should be kept or deleted. "
        "Asking the user is the default behavior."
    ),
)
@click.option(
    "--local_override_policy",
    type=click.Choice(["keep_local", "override", "ask"]),
    default="ask",
    help=(
        "Defines whether repositories existing locally AND "
        "in different version or with different URI in the config file,"
        "should be kept in local version or be overridden"
        "Asking the user is the default behavior."
    ),
)
@click.option(
    "--ignore_catkin",
    default=False,
    is_flag=True,
    help="Prevent catkin workspace from getting adapted",
)
@click.option(
    "--ignore_colcon",
    default=False,
    is_flag=True,
    help="Prevent colcon workspace from getting adapted",
)
@click.option(
    "--ignore_misc",
    default=False,
    is_flag=True,
    help="Prevent misc workspace from getting adapted",
)
@click.option(
    "--no_submodules",
    default=False,
    is_flag=True,
    help="Prevent git submodules from being cloned",
)
@click.pass_context
def cli(
    ctx,
    local_delete_policy,
    ignore_catkin,
    ignore_colcon,
    ignore_misc,
    no_submodules,
    local_override_policy,
):
    """Adapts an environment to given config file.
    New repositories will be added, versions/branches will be changed and
    deleted repositories will/may be removed.
    Provide path to config file as [ARGS] after the environment name.
    """

    print(
        "The policy for deleting repos only existing locally is: '{}'".format(
            local_delete_policy
        )
    )

    if ctx.invoked_subcommand is None:
        click.echo(
            "No environment specified. Please choose one "
            "of the available environments!"
        )
