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
import os

import pytest
import subprocess

from click.testing import CliRunner

import robot_folders.helpers.directory_helpers as directory_helpers
import robot_folders.helpers.ros_version_helpers as ros_versions
import robot_folders.commands.add_environment as add_environment
import robot_folders.commands.delete_environment as delete_environment

from .fixture_ros_installation import fake_ros_installation


@pytest.mark.usefixtures("fake_ros_installation")
def test_add_catkin(mocker):
    mocker.patch("subprocess.check_call")

    runner = CliRunner()

    installed_ros_distros = sorted(ros_versions.installed_ros_1_versions())
    ros_distro = installed_ros_distros[-1]

    result = runner.invoke(
        add_environment.cli,
        " ".join(
            [
                "--create_catkin=yes",
                "--create_misc_ws=no",
                "--create_colcon=no",
                "--copy_cmake_lists=no",
                "--underlays=skip",
                f"--ros_distro={ros_distro}",
                "testing_ws",
            ]
        ),
    )
    print(result.output)
    assert result.exit_code == 0
    catkin_dir = directory_helpers.get_catkin_dir(
        os.path.join(directory_helpers.get_checkout_dir(), "testing_ws")
    )
    subprocess.check_call.assert_called_once_with(
        [
            "bash",
            "-c",
            "source /opt/ros/noetic/setup.bash && catkin_make   -DCMAKE_EXPORT_COMPILE_COMMANDS=1",
        ],
        cwd=catkin_dir,
    )
    assert os.path.isdir(catkin_dir)

    result = runner.invoke(delete_environment.cli, "--force testing_ws")
    assert result.exit_code == 0
    assert os.path.isdir(catkin_dir) is False


@pytest.mark.usefixtures("fake_ros_installation")
def test_add_colcon(mocker):
    mocker.patch("subprocess.check_call")

    runner = CliRunner()

    installed_ros_distros = sorted(ros_versions.installed_ros_2_versions())
    ros_distro = installed_ros_distros[-1]

    result = runner.invoke(
        add_environment.cli,
        " ".join(
            [
                "--create_catkin=no",
                "--create_misc_ws=no",
                "--create_colcon=yes",
                "--copy_cmake_lists=no",
                "--underlays=skip",
                f"--ros2_distro={ros_distro}",
                "testing_ws",
            ]
        ),
    )
    print(result.output)
    assert result.exit_code == 0

    colcon_dir = directory_helpers.get_colcon_dir(
        os.path.join(directory_helpers.get_checkout_dir(), "testing_ws")
    )
    subprocess.check_call.assert_called_once_with(
        [
            "bash",
            "-c",
            "source /opt/ros/rolling/setup.bash && colcon build --symlink-install --cmake-args   -DCMAKE_EXPORT_COMPILE_COMMANDS=1",
        ],
        cwd=colcon_dir,
        env=os.environ.copy(),
    )
    assert os.path.isdir(colcon_dir)

    result = runner.invoke(delete_environment.cli, "--force testing_ws")
    assert result.exit_code == 0
    assert os.path.isdir(colcon_dir) is False
