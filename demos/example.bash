#!/bin/bash

###############################################################
#
# This file is part of the robot_folders repository system.
#
# It is an example for how a demo bash script should be 
# created. You can use it as a template for your own one.
#
#
# From the readme.txt we know:
#
# A demo should be runable by one script and without any
# additional knowledge.
#
# Every demo consists of one script that has the following
# purpose:
#
# 1. Print out a summary about the demo and also what you shall
#    take care about or how to handle problems.
# 2. Choose the environment to work in, especially choose a 
#    folder in ~/checkout.
# 3. Run a set of executables, scripts and/or launch files.
#
###############################################################

echo "Running EXAMPLE demo ..."
echo ""

# get this script's directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


# 1. Print out a summary about the demo and also what you shall
#    take care about or how to handle problems.
echo "########################################################################"
echo "#"
echo "# This demo doesn't do anything useful. You can just see how a"
echo "# demo script usually works."
echo "#"
echo "# Take care that you are prepared to look at the output of the screen."
echo "# There is no extra program required. You could anyway take care that "
echo "# all sensors are up and running but it's not important."
echo "#"
echo "# If the demo fails resulting in some headache it's a good idea to open"
echo "# a window to get some fresh air."
echo "#"
echo "########################################################################"


# 2. Choose the environment to work in, especially choose a
#    folder in ~/checkout.
source $SCRIPT_DIR/../checkout/example/setup.bash


# 3. Run a set of executables, scripts and/or launch files.
example_executable.bash


echo "... demo stopped"
