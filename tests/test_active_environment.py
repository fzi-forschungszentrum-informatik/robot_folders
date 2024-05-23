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

from click.testing import CliRunner

import robot_folders.commands.active_environment
from robot_folders.helpers.directory_helpers import get_checkout_dir


def test_active_environment():
    runner = CliRunner()

    # First: Try to get the latest active env
    env_file = os.path.join(get_checkout_dir(), ".cur_env")
    with open(env_file, "w") as file_content:
        file_content.write("testing_ws")
    result = runner.invoke(robot_folders.commands.active_environment.cli)
    assert (
        result.output
        == "No active environment. Last activated environment: testing_ws\n"
    )
    assert result.exit_code == 0

    # Set an active env
    os.environ["ROB_FOLDERS_ACTIVE_ENV"] = "testing_ws"
    result = runner.invoke(robot_folders.commands.active_environment.cli)
    assert result.output == "Active environment: testing_ws\n"
    assert result.exit_code == 0
