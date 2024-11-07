#!/bin/bash
cd `dirname $0` 

# go to workspace
cd ..

# This script installs dependencies for ROS 2 packages using the rosdep tool.
# 
# Options:
# -i: Installs the dependencies listed in the package.xml files of the packages in the specified path.
# --from-path src: Specifies the source directory containing the ROS 2 packages.
# --rosdistro foxy: Specifies the ROS 2 distribution to use (in this case, Foxy).
# -y: Automatically confirms the installation of dependencies without prompting the user.
rosdep install -i --from-path src --rosdistro foxy -y

# Install f1tenth_gym 
cd src/simulation/
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && pip3 install -e .
cd f1tenth_gym
pip3 install -e .