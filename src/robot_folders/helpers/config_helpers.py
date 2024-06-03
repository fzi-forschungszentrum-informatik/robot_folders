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
"""Module for reading and handling the config."""
from __future__ import print_function
import os
import shutil
import sys
import yaml

from importlib import resources

import robot_folders.helpers.resources

XDG_CONFIG_HOME = os.getenv(
    "XDG_CONFIG_HOME", os.path.expandvars(os.path.join("$HOME", ".config"))
)
FILENAME_USERCONFIG = os.path.join(XDG_CONFIG_HOME, "robot_folders.yaml")


def get_resource_path(filename: str):
    if sys.version_info.major == 3 and sys.version_info.minor < 9:
        with resources.path(".".join([str(__package__), "resources"]), filename) as p:
            return str(p.as_posix())
    return str(resources.files(robot_folders.helpers.resources).joinpath(filename))


class Userconfig(object):
    """Class for managing a userconfig"""

    config = None
    config_fallback = None
    initialized = False

    @classmethod
    def init_class(cls):
        """Load the distribution config file"""
        filename_distribute = get_resource_path("userconfig_distribute.yaml")
        with open(filename_distribute) as p:
            file_content = p.read()
            try:
                Userconfig.config_fallback = yaml.safe_load(file_content)
            except yaml.YAMLError as exc:
                print("Error in configuration file:", exc)
            except IOError as exc:
                print("ERROR: There was a problem loading the distribution file:", exc)

        # Load the user-modified config file
        try:
            with open(FILENAME_USERCONFIG, "r") as file_content:
                try:
                    Userconfig.config = yaml.safe_load(file_content)
                except yaml.YAMLError as exc:
                    print("Error in configuration file:", exc)
        except (IOError, FileNotFoundError) as exc:
            print("Did not find userconfig file. Copying the distribution file.")
            Userconfig.config = Userconfig.config_fallback
            if not os.path.exists(XDG_CONFIG_HOME):
                os.makedirs(XDG_CONFIG_HOME)
            shutil.copy(filename_distribute, FILENAME_USERCONFIG)
        Userconfig.initialized = True


# This is the internal yaml query. It can be used for the modified
# or the distribution config
def _get_value_safe(dictionary, section, value, debug=True):
    result = None
    try:
        result = dictionary[section][value]
    except KeyError:
        if debug:
            print("Did not find key {}.{}!!!".format(section, value))
    except TypeError:
        print(
            "There is an illegal config. "
            "Probably something went wrong during initialization. "
            "Check your config file."
        )
    return result


# Interface funtion to the outside. You might want to consider the
def get_value_safe(section, value, debug=True):
    """Retrieve an entry from the config backend.

    Will try to find the given entry in the config. If it is not
    found in the user-modified config, the default value from the
    distributed config will be used.
    If it is not found at all, a None object will be returned.

    - **parameters**, **types**, **return** and **return types**::
        :param section: The top level key for the searched entry
        :param value: The second level key for the searched entry
        :param debug: Print debug output when searching for the entry (default True)
        :type section: string
        :type value: string
        :type debug: bool
        :returns: config entry
        :rtype: mixed
    """
    if not Userconfig.initialized:
        Userconfig.init_class()

    result = _get_value_safe(Userconfig.config, section, value, debug=False)
    if result is None:
        result = _get_value_safe(Userconfig.config_fallback, section, value, debug)
    return result


def get_value_safe_default(section, value, default, debug=True):
    """Retrieve an entry from the config backend or use default value.

    Will try to find the given entry in the config. If it is not
    found in the user-modified config, the default value from the
    distributed config will be used.
    If it is not found at all, the default value will be returned.

    - **parameters**, **types**, **return** and **return types**::
        :param section: The top level key for the searched entry
        :param value: The second level key for the searched entry
        :param default: The default entry if not found
        :param debug: Print debug output when searching for the entry (default True)
        :type section: string
        :type value: string
        :type default: mixed, the same as return type
        :type debug: bool
        :returns: config entry
        :rtype: mixed
    """
    result = get_value_safe(section, value, debug)
    if result is None:
        result = default

    return result
