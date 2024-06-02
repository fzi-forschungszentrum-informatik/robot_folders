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

import robot_folders.helpers.ros_version_helpers as ros


@pytest.fixture
def fake_ros_installation(fs):
    fs.create_file("/opt/ros/melodic/setup.sh", contents="catkin")
    fs.create_file("/opt/ros/noetic/setup.sh", contents="catkin")
    fs.create_file("/opt/ros/humble/setup.sh", contents="AMENT_CURRENT_PREFIX")
    fs.create_file("/opt/ros/jazzy/setup.sh", contents="COLCON_CURRENT_PREFIX")
    fs.create_file("/opt/ros/rolling/setup.sh", contents="AMENT_CURRENT_PREFIX")
    fs.create_file("/opt/ros/spy/setup.sh", contents="imnotros")
    fs.create_file("/opt/foo/setup.sh", contents="imnotros")
    yield fs


@pytest.mark.usefixtures("fake_ros_installation")
def test_installed_ros1_versions():
    assert ros.installed_ros_1_versions() == ["melodic", "noetic"]


@pytest.mark.usefixtures("fake_ros_installation")
def test_installed_ros2_versions():
    assert ros.installed_ros_2_versions() == ["humble", "jazzy", "rolling"]


@pytest.mark.usefixtures("fake_ros_installation")
def test_installed_ros_distros():
    assert ros.installed_ros_distros() == [
        "melodic",
        "noetic",
        "humble",
        "jazzy",
        "rolling",
    ]
