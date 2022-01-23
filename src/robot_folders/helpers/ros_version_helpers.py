"""Little helper around ROS versions"""

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
        if 'ament' in open('/opt/ros/' + distro + '/setup.sh').read():
            installed_ros_distros.append(distro)
    return installed_ros_distros
