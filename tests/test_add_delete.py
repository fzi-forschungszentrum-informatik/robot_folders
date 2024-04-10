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
import subprocess
import sys
import unittest
import pytest

import os

import robot_folders.helpers.config_helpers as config_helpers
import robot_folders.helpers.directory_helpers as directory_helpers
import robot_folders.helpers.ros_version_helpers as ros_versions


class TestCLI(unittest.TestCase):
    def test_add_catkin_only(self):
        installed_ros_distros = sorted(ros_versions.installed_ros_1_versions())
        if len(installed_ros_distros) == 0:
            raise unittest.SkipTest(
                "Skipping this test since no ROS 1 distro is installed."
            )
        print("Available ROS distributions: {}".format(installed_ros_distros))
        ros_distro = installed_ros_distros[-1]
        try:
            process_result = subprocess.check_call(
                [
                    "rob_folders",
                    "add_environment",
                    "--create_catkin=yes",
                    "--create_misc_ws=no",
                    "--create_colcon=no",
                    "--copy_cmake_lists=no",
                    "--ros_distro={}".format(ros_distro),
                    "testing_ws",
                ]
            )
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

        catkin_dir = directory_helpers.get_catkin_dir(
            os.path.join(directory_helpers.get_checkout_dir(), "testing_ws")
        )

        self.assertTrue(os.path.isdir(catkin_dir))
        # self.assertTrue(os.path.exists(os.path.join(catkin_dir, "src", "CMakeLists.txt")))

        # cleanup
        try:
            process_result = subprocess.check_call(
                ["rob_folders", "delete_environment", "--force", "testing_ws"]
            )
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

    def test_add_colcon_only(self):
        installed_ros_distros = sorted(ros_versions.installed_ros_2_versions())
        if len(installed_ros_distros) == 0:
            raise unittest.SkipTest(
                "Skipping this test since no ROS 1 distro is installed."
            )
        ros_distro = installed_ros_distros[-1]
        try:
            process_result = subprocess.check_call(
                [
                    "rob_folders",
                    "add_environment",
                    "--create_catkin=no",
                    "--create_misc_ws=no",
                    "--create_colcon=yes",
                    "--copy_cmake_lists=no",
                    "--ros2_distro={}".format(ros_distro),
                    "testing_ws",
                ]
            )
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

        colcon_dir = directory_helpers.get_colcon_dir(
            os.path.join(directory_helpers.get_checkout_dir(), "testing_ws")
        )

        self.assertTrue(os.path.isdir(colcon_dir))

        # cleanup
        try:
            process_result = subprocess.check_call(
                ["rob_folders", "delete_environment", "--force", "testing_ws"]
            )
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)
