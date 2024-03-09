#!/usr/bin/env python3
"""
This file is kept for legacy pip compatibility only.
"""

from setuptools import setup

setup(
    name='robot_folders',
    version='0.3.1',
    python_requires='>=3',
    packages = [
        "robot_folders",
        "robot_folders.commands",
        "robot_folders.helpers",
        "robot_folders.helpers.resources"],
    package_dir = {"": "src"},
    install_requires=[
        'Click>=7.0',
        'gitpython',
        'PyYaml',
        'vcstool'
    ],
    entry_points='''
        [console_scripts]
        rob_folders=robot_folders.main:cli
    ''',
    scripts=[
        "bin/rob_folders_get_source_command.py",
        "bin/rob_folders-complete.sh",
        "bin/rob_folders_source.sh",
        "bin/source_environment.sh",
        ],
    include_package_data=True,
    package_data={"": ["*.yaml"]}
)
