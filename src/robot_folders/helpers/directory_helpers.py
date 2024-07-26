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
"""Helpers around handling folders"""
from __future__ import print_function
import os
import errno
import getpass
import subprocess

import click

import robot_folders.helpers.config_helpers as config_helpers


def get_base_dir():
    """Returns the robot_folders base dir."""
    base_dir = os.environ["ROB_FOLDERS_BASE_DIR"]
    return os.path.realpath(base_dir)


def get_last_activated_env():
    """Looks for the most recently sourced environment"""
    env_file = os.path.join(get_checkout_dir(), ".cur_env")

    if os.path.isfile(env_file):
        with open(env_file, "r") as file_content:
            return file_content.read().rstrip()
    else:
        print(
            "No recently activated environment found. Is this your first run?"
            "Try to add an environment and then do a change_environment to this."
        )
        return None


def get_active_env():
    """Returns the currently sourced environment. If none is sourced, this will return None"""
    try:
        active_env = os.environ["ROB_FOLDERS_ACTIVE_ENV"]
        return active_env
    except KeyError:
        # print "Currently, there is no active environment!\n\
        # To get rid of this message, please source the most recent environment or \
        # change to another one."
        return None


def get_active_env_path():
    """Returns the path of the currently sourced environment"""
    active_env = get_active_env()
    if active_env is None:
        active_env_fallback = get_last_activated_env()
        if active_env_fallback is None:
            return None
        active_env = active_env_fallback
    return os.path.join(get_checkout_dir(), active_env)


def mkdir_p(path):
    """Checks whether a directory exists, otherwise it will be created."""
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


def recursive_rmdir(path):
    """Recursively deletes a path"""
    for i in os.listdir(path):
        item = os.path.join(path, i)
        if os.path.isdir(item):
            recursive_rmdir(item)
        else:
            os.remove(item)
    os.rmdir(path)


def get_checkout_dir():
    """Get the robot folders checkout directory from the userconfig"""
    checkout_config = config_helpers.get_value_safe(
        "directories", "checkout_dir", debug=False
    )
    if checkout_config == "" or checkout_config is None:
        checkout_config = "~/checkout"
    expanded = os.path.expanduser(checkout_config)
    if not os.path.exists(expanded):
        mkdir_p(expanded)
    return expanded


def get_catkin_dir(env_dir=""):
    """Tries to find the right catkin workspace in the Currently \
    sourced environment."""

    path = ""
    cur_env_path = env_dir
    if env_dir == "":
        cur_env_path = get_active_env_path()

    valid_names = config_helpers.get_value_safe_default(
        "directories", "catkin_names", ["catkin_workspace", "catkin_ws"], debug=False
    )
    for path_name in valid_names:
        path = os.path.join(cur_env_path, path_name)
        if os.path.exists(path):
            return path
    return os.path.join(cur_env_path, "catkin_ws")


def get_colcon_dir(env_dir=""):
    """Tries to find the right colcon workspace in the Currently \
    sourced environment."""

    path = ""
    cur_env_path = env_dir
    if env_dir == "":
        cur_env_path = get_active_env_path()

    valid_names = config_helpers.get_value_safe_default(
        "directories",
        "colcon_names",
        ["colcon_workspace", "colcon_ws", "dev_ws"],
        debug=False,
    )
    for path_name in valid_names:
        path = os.path.join(cur_env_path, path_name)
        if os.path.exists(path):
            return path
    return os.path.join(cur_env_path, "colcon_ws")


def get_misc_dir(env_dir=""):
    """Tries to find the right misc workspace in the Currently \
    sourced environment."""

    path = ""
    cur_env_path = env_dir
    if env_dir == "":
        cur_env_path = get_active_env_path()

    valid_names = ["misc_ws"]
    for path_name in valid_names:
        path = os.path.join(cur_env_path, path_name)
        if os.path.exists(path):
            return path
    return os.path.join(cur_env_path, "misc_ws")


def yes_no_to_bool(bool_str):
    """
    Converts a yes/no string to a bool
    """
    return bool_str == "yes" or bool_str == "Yes"


def check_build_on_nobackup(local_build="ask"):
    """
    Checks whether there is a nobackup on this system. If there is, the local_build
    parameter is used to determine whether the no_backup folder should be used or
    the local structure should be used.

    local_build == 'yes' - ignore nobackup even if it exists -> will return false
    """
    # If the no_backup location exists, offer to build in no_backup
    has_nobackup = False
    no_backup_location = os.path.expanduser(
        config_helpers.get_value_safe("directories", "no_backup_dir")
    )
    try:
        if os.path.isdir(no_backup_location):
            has_nobackup = True
    except subprocess.CalledProcessError:
        pass

    if has_nobackup:
        if local_build == "ask":
            build_dir_choice = click.prompt(
                "Which folder should I use as a base for creating the build tree?\n"
                "Type 'local' for building inside the local checkout tree.\n"
                "Type 'no_backup' (or simply press enter) for building in the no_backup "
                "space.\n",
                type=click.Choice(["no_backup", "local"]),
                default="no_backup",
            )
            build_dir_choice = build_dir_choice == "local"
        else:
            build_dir_choice = yes_no_to_bool(local_build)

        return not build_dir_choice


def get_build_base_dir(use_no_backup):
    """
    Gets the base directory for building depending on whether no_backup should be used or not
    """
    if use_no_backup:
        no_backup_dir = config_helpers.get_value_safe("directories", "no_backup_dir")
        build_base_dir = os.path.expanduser(f"{no_backup_dir}/robot_folders_build_base")
    else:
        build_base_dir = get_checkout_dir()

    return build_base_dir


def is_fzirob_environment(checkout_folder, env_dir):
    """Checks whether a given directory actually contains an environment"""
    is_environment = False

    environment_folders = config_helpers.get_value_safe_default(
        section="directories", value="catkin_names", default=["catkin_workspace"]
    )
    environment_folders = environment_folders + config_helpers.get_value_safe_default(
        section="directories", value="colcon_names", default=["colcon_workspace"]
    )
    environment_files = ["setup.bash", "setup.zsh", "setup.sh"]

    possible_env = os.path.join(checkout_folder, env_dir)
    if os.path.isdir(possible_env):
        # check folders for existence
        for folder in environment_folders:
            if os.path.isdir(os.path.join(possible_env, folder)):
                is_environment = True
                break
        # check if folder was already found
        if not is_environment:
            # check files for existences
            for filename in environment_files:
                if os.path.exists(os.path.join(possible_env, filename)):
                    is_environment = True
                    break

    return is_environment


def list_environments():
    """List all environments"""
    checkout_folder = get_checkout_dir()
    return [
        env_dir
        for env_dir in sorted(os.listdir(checkout_folder))
        if is_fzirob_environment(checkout_folder, env_dir)
    ]
