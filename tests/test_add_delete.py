import subprocess
import sys
import unittest
import pytest

import os

import helpers.config_helpers as config_helpers
import helpers.directory_helpers as directory_helpers
import helpers.ros_version_helpers as ros_versions


class TestCLI(unittest.TestCase):
    def test_add_catkin_only(self):
        installed_ros_distros = sorted(ros_versions.installed_ros_1_versions())
        print("Available ROS distributions: {}".format(installed_ros_distros))
        ros_distro = installed_ros_distros[-1]
        try:
            process_result = subprocess.check_call(
                ["rob_folders",
                 "add_environment",
                 "--create_catkin=yes",
                 "--create_misc_ws=no",
                 "--create_colcon=no",
                 "--copy_cmake_lists=no",
                 "--ros_distro={}".format(ros_distro),
                 "testing_ws"])
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

        catkin_dir = directory_helpers.get_catkin_dir(
            os.path.join(directory_helpers.get_checkout_dir(), "testing_ws"))

        self.assertTrue(os.path.isdir(catkin_dir))
        # self.assertTrue(os.path.exists(os.path.join(catkin_dir, "src", "CMakeLists.txt")))

        # cleanup
        try:
            process_result = subprocess.check_call(
                ["rob_folders",
                 "delete_environment",
                 "--force",
                 "testing_ws"])
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)
    
    def test_add_colcon_only(self):
        installed_ros_distros = sorted(ros_versions.installed_ros_2_versions())
        ros_distro = installed_ros_distros[-1]
        try:
            process_result = subprocess.check_call(
                ["rob_folders",
                 "add_environment",
                 "--create_catkin=no",
                 "--create_misc_ws=no",
                 "--create_colcon=yes",
                 "--copy_cmake_lists=no",
                 "--ros2_distro={}".format(ros_distro),
                 "testing_ws"])
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

        colcon_dir = directory_helpers.get_colcon_dir(
            os.path.join(directory_helpers.get_checkout_dir(), "testing_ws"))

        self.assertTrue(os.path.isdir(colcon_dir))

        # cleanup
        try:
            process_result = subprocess.check_call(
                ["rob_folders",
                 "delete_environment",
                 "--force",
                 "testing_ws"])
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)
