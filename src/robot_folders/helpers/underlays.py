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

import inquirer

import robot_folders.helpers.directory_helpers as dir_helpers


class UnderlayManager:
    def __init__(self, env_name):
        self.env_name = env_name
        self.underlays = self.read_underlay_file()

    def query_underlays(self, active_list=None):
        """CLI interface to update the underlay_list"""
        envs = dir_helpers.list_environments()
        if envs:
            questions = [
                inquirer.Checkbox(
                    "underlays",
                    message="Which environments would you like to use as underlays (Can be left empty)?",
                    choices=envs,
                    default=active_list,
                ),
            ]
            self.underlays = inquirer.prompt(questions)["underlays"]

    def _get_underlay_filename(self):
        return os.path.join(
            dir_helpers.get_checkout_dir(), self.env_name, "underlays.txt"
        )

    def read_underlay_file(self):
        underlays = []
        if os.path.exists(self._get_underlay_filename()):
            with open(
                self._get_underlay_filename(), encoding="utf-8", mode="r"
            ) as underlay_file:
                lines = underlay_file.readlines()
                for line in lines:
                    underlays.append(os.path.basename(line).strip())
        return underlays

    def write_underlay_file(self):
        with open(
            self._get_underlay_filename(), encoding="utf-8", mode="w"
        ) as underlay_file:
            for underlay in self.underlays:
                underlay_file.write(
                    os.path.join(dir_helpers.get_checkout_dir(), underlay) + "\n"
                )
