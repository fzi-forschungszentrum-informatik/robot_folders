#!/usr/bin/env python3
"""
This file is kept for legacy pip compatibility only.
"""

from setuptools import setup

# read the contents of your README file
from pathlib import Path

this_directory = Path(__file__).parent
long_description = (this_directory / "README.rst").read_text()

setup(
    name="robot_folders",
    version="0.5.0",
    python_requires=">=3",
    description="robot_folders is your workspace handling utility around the ROS ecosystem.",
    long_description=long_description,
    long_description_content_type="text/x-rst",
    packages=[
        "robot_folders",
        "robot_folders.commands",
        "robot_folders.helpers",
        "robot_folders.helpers.resources",
    ],
    package_dir={"": "src"},
    install_requires=["Click>=8.0", "gitpython", "inquirer", "PyYaml", "vcstool"],
    entry_points="""
        [console_scripts]
        rob_folders=robot_folders.main:cli
    """,
    scripts=[
        "bin/rob_folders_get_source_command.py",
        "bin/rob_folders-complete.sh",
        "bin/rob_folders_source.sh",
        "bin/source_environment.sh",
    ],
    include_package_data=True,
    package_data={"": ["*.yaml"]},
    license_files=("LICENSE.txt"),
)
