import subprocess
import sys
import unittest
import pytest

import os

import helpers.config_helpers as config_helpers
import helpers.directory_helpers as directory_helpers


class TestCLI(unittest.TestCase):
    def test_rob_folders_runs(self):
        try:
            subprocess.run(["rob_folders"], stdout=subprocess.PIPE)
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

    def test_get_checkout_base_dir(self):
        try:
            process_result = subprocess.run(["rob_folders", "get_checkout_base_dir"],
                                            stdout=subprocess.PIPE, check=True)
        except:
            (etype, evalue, etrace) = sys.exc_info()
            self.fail("Failed with %s" % evalue)

        expected = config_helpers.get_value_safe("directories", "checkout_dir")

        if expected:
            self.assertEqual(process_result.stdout.strip(), bytes(expected, 'utf-8'))
        else:
            self.assertEqual(process_result.stdout.strip(),
                             os.path.expanduser(b"~/robot_folders/checkout"))

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

        # Cleanup is currently skipped as the delete command doesn't support a headless call
        # # cleanup
        # try:
            # process_result = subprocess.check_call(
                # ["rob_folders",
                 # "delete_environment",
                 # "testing_ws"])
        # except:
            # (etype, evalue, etrace) = sys.exc_info()
            # self.fail("Failed with %s" % evalue)


