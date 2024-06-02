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
import pytest

import os

import click
from click.testing import CliRunner

import robot_folders.helpers.directory_helpers as dir_helpers


def test_get_last_activated_env(fs):

    # First without an env file
    assert dir_helpers.get_last_activated_env() is None

    # Now, with an env file
    env_file = os.path.join(dir_helpers.get_checkout_dir(), ".cur_env")
    fs.create_file(env_file, contents="schnutzelfu")
    assert dir_helpers.get_last_activated_env() == "schnutzelfu"


def test_get_active_env_path(fs):

    os.environ.pop("ROB_FOLDERS_ACTIVE_ENV", None)
    assert dir_helpers.get_active_env_path() == None

    env_file = os.path.join(dir_helpers.get_checkout_dir(), ".cur_env")
    fs.create_file(env_file, contents="schnutzelfu")
    expected = os.path.join(dir_helpers.get_checkout_dir(), "schnutzelfu")
    assert dir_helpers.get_active_env_path() == expected

    os.environ["ROB_FOLDERS_ACTIVE_ENV"] = "idoexist"
    expected = os.path.join(dir_helpers.get_checkout_dir(), "idoexist")
    assert dir_helpers.get_active_env_path() == expected


def test_directory_handling(fs):

    test_folder = "/tmp/foo/bar/goo"
    test_file = os.path.join(test_folder, "myfile")

    # Execute once
    dir_helpers.mkdir_p(test_folder)
    # Execute another time, should also succeed
    dir_helpers.mkdir_p(test_folder)

    with open(test_file, "w") as f:
        f.write("hello")

    # Calling mkdir_p on an existing file should raise an exception
    with pytest.raises(OSError):
        dir_helpers.mkdir_p(test_file)

    # Make sure everything exists
    assert os.path.isfile(test_file)
    assert os.path.isdir(test_folder)

    dir_helpers.recursive_rmdir("/tmp/foo")

    # Make sure test objects don't exist anymore
    assert not os.path.isfile(test_file)
    assert not os.path.isdir(test_folder)


def test_nobackup(fs):

    @click.command()
    @click.option("--local_build")
    def test_command(local_build):
        return dir_helpers.check_build_on_nobackup(local_build)

    no_backup = os.path.expanduser("~/no_backup")

    # Without no_backup it should return None
    runner = CliRunner()
    result = runner.invoke(
        test_command,
        args=["--local_build=ask"],
        input="no_backup",
        standalone_mode=False,
    )
    assert not result.exception
    assert result.return_value == None

    # With no_backup and answered "no_backup" it should return True
    dir_helpers.mkdir_p(no_backup)
    runner = CliRunner()
    result = runner.invoke(
        test_command,
        args=["--local_build=ask"],
        input="no_backup",
        standalone_mode=False,
    )
    assert result.exit_code == 0
    assert result.return_value == True

    # With no_backup and answered "local" it should return False
    dir_helpers.mkdir_p(no_backup)
    runner = CliRunner()
    result = runner.invoke(
        test_command, args=["--local_build=ask"], input="local", standalone_mode=False
    )
    assert result.exit_code == 0
    assert result.return_value == False

    # With no_backup and answered anything it should return True
    dir_helpers.mkdir_p(no_backup)
    runner = CliRunner()
    result = runner.invoke(
        test_command,
        args=["--local_build=ask"],
        input="schuigfdf",
        standalone_mode=False,
    )
    assert result.exit_code == 0
    assert result.return_value == True

    # When passing local_build=no it should return True
    runner = CliRunner()
    result = runner.invoke(
        test_command,
        args=["--local_build=no"],
        input="no_backup",
        standalone_mode=False,
    )
    assert result.exit_code == 0
    assert result.return_value == True

    # When passing local_build=yes it should return False
    runner = CliRunner()
    result = runner.invoke(
        test_command,
        args=["--local_build=yes"],
        input="no_backup",
        standalone_mode=False,
    )
    assert result.exit_code == 0
    assert result.return_value == False
