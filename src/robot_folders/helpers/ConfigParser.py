"""This module help parsing environment config files"""
import click

import yaml

class ConfigFileParser(object):
    """Parser for robot_folders environment configs"""
    def __init__(self, config_file_name):
        with open(config_file_name, 'r') as file_content:
            self.data = yaml.load(file_content, Loader=yaml.SafeLoader)
        click.echo('The following config file is passed:\n{}'.format(self.data))

    def parse_misc_ws_config(self):
        """Parses the misc_ws part of the data"""
        has_misc_ws = False
        misc_ws_rosinstall = None
        if 'misc_ws' in self.data:
            has_misc_ws = True
            if 'rosinstall' in self.data['misc_ws']:
                misc_ws_rosinstall = self.data['misc_ws']['rosinstall']
        return has_misc_ws, misc_ws_rosinstall

    def parse_ros_config(self):
        """Parses the catkin_workspace part of the data"""
        ros_rosinstall = None
        has_catkin = False
        if "catkin_workspace" in self.data:
            has_catkin = True
            if "rosinstall" in self.data["catkin_workspace"]:
                ros_rosinstall = self.data["catkin_workspace"]["rosinstall"]

        return has_catkin, ros_rosinstall

    def parse_ros2_config(self):
        """Parses the colcon_workspace part of the data"""
        ros2_rosinstall = None
        has_colcon = False
        if "colcon_workspace" in self.data:
            has_colcon = True
            if "rosinstall" in self.data["colcon_workspace"]:
                ros2_rosinstall = self.data["colcon_workspace"]["rosinstall"]

        return has_colcon, ros2_rosinstall

    def parse_demo_scripts(self):
        """Parses the demos part of the data"""
        script_list = dict()
        if 'demos' in self.data:
            for script in self.data['demos']:
                script_list[script] = self.data['demos'][script]
        return script_list
