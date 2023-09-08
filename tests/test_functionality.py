import subprocess
import sys
import unittest
import pytest

import os

import robot_folders.helpers.config_helpers as config_helpers
import robot_folders.helpers.directory_helpers as directory_helpers


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

        expected = directory_helpers.get_checkout_dir()

        self.assertEqual(process_result.stdout.strip(), bytes(expected, 'utf-8'))
