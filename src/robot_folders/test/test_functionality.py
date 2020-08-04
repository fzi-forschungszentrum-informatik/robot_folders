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
