import subprocess
import sys
import unittest
import pytest

import os

import helpers.config_helpers as config_helpers
import helpers.directory_helpers as directory_helpers

class TestCLI(unittest.TestCase):
    def test_add_catkin_only(self):
        try:
            process_result = subprocess.check_call(
                ["rob_folders",
                 "add_environment",
                 "--create_ic=no",
                 "--create_mca=no",
                 "--create_catkin=yes",
                 "--create_misc_ws=no",
                 "--copy_cmake_lists=no",
                 "testing_ws"])
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

        catkin_dir = directory_helpers.get_catkin_dir(
            os.path.join(directory_helpers.get_checkout_dir(), "testing_ws"))

        self.assertTrue(os.path.isdir(catkin_dir))
        self.assertTrue(os.path.exists(os.path.join(catkin_dir, "src", "CMakeLists.txt")))

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

    def test_add_ic_only(self):
        try:
            process_result = subprocess.check_call(
                ["rob_folders",
                 "add_environment",
                 "--create_ic=yes",
                 "--create_mca=no",
                 "--create_catkin=no",
                 "--create_misc_ws=no",
                 "--copy_cmake_lists=no",
                 "testing_ws"])
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

        ic_dir = os.path.join(directory_helpers.get_checkout_dir(), "testing_ws", "ic_workspace")

        self.assertTrue(os.path.isdir(ic_dir))
        self.assertTrue(os.path.exists(os.path.join(ic_dir, "CMakeLists.txt")))

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

    # def test_add_mca_only(self):
        # try:
            # process_result = subprocess.check_call(
            # ["rob_folders",
            # "add_environment",
            # "--create_ic=no",
            # "--create_mca=yes",
            # "--create_catkin=no",
            # "--copy_cmake_lists=no",
            # "testing_ws"])
        # except:
            # (etype, evalue, etrace) = sys.exc_info()
            # self.fail("Failed with %s" % evalue)

        # mca_dir = os.path.join(directory_helpers.get_checkout_dir(), "testing_ws", "mca_workspace")

        # self.assertTrue(os.path.isdir(mca_dir))
        # self.assertTrue(os.path.exists(os.path.join(mca_dir, "CMakeLists.txt")))

        # # cleanup
        # try:
            # process_result = subprocess.check_call(
            # ["rob_folders",
            # "delete_environment",
            # "--force",
            # "testing_ws"])
        # except:
            # (etype, evalue, etrace) = sys.exc_info()
            # self.fail("Failed with %s" % evalue)
