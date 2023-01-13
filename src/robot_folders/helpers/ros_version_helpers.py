"""Little helper around ROS versions"""

import subprocess
import os

def installed_ros_distros():
    """Returns all installed ROS versions (ROS1 and ROS2)"""
    return os.listdir("/opt/ros")

def installed_ros_1_versions():
    """Returns a list of all installed ROS1 versions"""
    #Check the setup if it contains catkin the ROS Build system
    temp_installed_ros_distros = os.listdir("/opt/ros")
    installed_ros_distros = []
    for distro in temp_installed_ros_distros:
        if 'catkin' in open('/opt/ros/' + distro + '/setup.sh').read():
            installed_ros_distros.append(distro)
    return installed_ros_distros

def installed_ros_2_versions():
    """Returns a list of all installed ROS2 versions"""
    #Check the setup if it contains ament the ROS2 Build system
    temp_installed_ros_distros = os.listdir("/opt/ros")
    installed_ros_distros = []
    for distro in temp_installed_ros_distros:
        source_script_path = os.path.join('/opt/ros', distro, 'setup.sh')
        if 'AMENT_PREFIX_PATH' in shell_source_env(source_script_path):
            installed_ros_distros.append(distro)
    return installed_ros_distros

def shell_source_env(source_script_path):
    """Emulates sourcing a shell file and returns the sourced environment as dictionary."""
    pipe = subprocess.Popen(". %s; env" % source_script_path, stdout=subprocess.PIPE, shell=True)
    output = pipe.communicate()[0].decode("utf-8")

    key_values = (x.split("=", 1) for x in output.splitlines())
    filtered_pairs = filter(lambda x: len(x) == 2, key_values)

    return dict(filtered_pairs)
