import click

from yaml import load as yaml_load

try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


class ConfigFileParser():
    def __init__(self, config_file_name):
        with open(config_file_name, 'r') as f:
            self.data = yaml_load(f, Loader=Loader)
        click.echo('The following config file is passed:\n{}'.format(self.data))

    def parse_ic_config(self):
        ic_rosinstall = None
        ic_packages = None
        ic_package_versions = None
        ic_grab_flags = []
        has_ic = False
        if 'ic_workspace' in self.data:
            has_ic = True
            if 'rosinstall' in self.data['ic_workspace']:
                ic_rosinstall = self.data['ic_workspace']['rosinstall']
                ic_packages = None

            if 'packages' in self.data['ic_workspace']:
                ic_packages = ' '.join(self.data['ic_workspace']['packages'])
                # if the base package is not specified in the config_file, add it nevertheless.
                if "base" not in ic_packages:
                    ic_packages = "base " + ic_packages
                    click.echo("The base-package for the ic_workspace has not been specified in the"
                               " configuration. It will be added regardless.")
                if ('package_versions' in self.data['ic_workspace'] and
                        self.data['ic_workspace']['package_versions'] is not None):
                    ic_package_versions = self.data['ic_workspace']['package_versions']

                if ('flags' in self.data['ic_workspace'] and
                        self.data['ic_workspace']['flags'] is not None):
                    for flag in self.data['ic_workspace']['flags']:
                        ic_grab_flags.append("--" + flag)

                ic_rosinstall = None
        return has_ic, ic_rosinstall, ic_packages, ic_package_versions, ic_grab_flags

    def parse_ros_config(self):
        ros_rosinstall = None
        has_catkin = False
        if "catkin_workspace" in self.data:
            has_catkin = True
            if "rosinstall" in self.data["catkin_workspace"]:
                ros_rosinstall = self.data["catkin_workspace"]["rosinstall"]

        return has_catkin, ros_rosinstall

    def parse_mca_config(self):
        has_mca = False
        if self.data['mca_workspace'] is not None:
            has_mca = True
            mca_additional_repos = self.data['mca_workspace']
        return has_mca, mca_additional_repos

    def parse_demo_scripts(self):
        script_list = dict()
        if 'demos' in self.data:
            for script in self.data['demos']:
                script_list[script] = self.data['demos'][script]
        return script_list
