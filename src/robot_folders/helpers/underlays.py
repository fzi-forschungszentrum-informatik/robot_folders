import os

import inquirer

import robot_folders.helpers.directory_helpers as dir_helpers

class UnderlayManager():
    def __init__(self, env_name):
        self.env_name = env_name
        self.underlays = self.read_underlay_file()

    def query_underlays(self, active_list=None):
        """CLI interface to update the underlay_list"""
        envs = dir_helpers.list_environments()
        if envs:
            questions = [
                inquirer.Checkbox('underlays',
                              message="Which environments would you like to use as underlays (Can be left empty)?",
                              choices=envs,
                              default=active_list,
                          ),
            ]
            self.underlays = inquirer.prompt(questions)["underlays"]

    def _get_underlay_filename(self):
        return os.path.join(dir_helpers.get_checkout_dir(), self.env_name, "underlays.txt")

    def read_underlay_file(self):
        underlays = []
        if os.path.exists(self._get_underlay_filename()):
            with open(self._get_underlay_filename(), encoding="utf-8", mode="r") as underlay_file:
                lines = underlay_file.readlines()
                for line in lines:
                    underlays.append(os.path.basename(line).strip())
        return underlays

    def write_underlay_file(self):
        with open(self._get_underlay_filename(),
                  encoding="utf-8",
                  mode='w') as underlay_file:
            for underlay in self.underlays:
                underlay_file.write(os.path.join(dir_helpers.get_checkout_dir(), underlay) + "\n")
