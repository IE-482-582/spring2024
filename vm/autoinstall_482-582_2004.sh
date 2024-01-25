#!/bin/bash

# ===============================================================================
# File:  autoinstall_482-582_2004.sh
# Updated: 2024-01-24
#
# NOTES:
#	* This script installs the necessary packages for ros noetic on Ubuntu 20.04
#
# ===============================================================================

sudo apt-get install build-essential
sudo reboot



set -e

# Create the user's "Projects" directory:
mkdir -p ${HOME}/Projects

# Prelims
cd ${HOME}

sudo apt-get --yes install git
sudo apt-get --yes remove modemmanager 
sudo apt --yes install net-tools

sudo apt --yes install python3-pip

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt --yes install curl 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt-get upgrade



# ---------------------------------------------------
# Github CLI
# SKIPPING

# ---------------------------------------------------
# ROS Noetic
echo ">>> Installing ROS Noetic..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt --yes install ros-noetic-desktop-full

sudo apt --yes install python3-rosdep
sudo rosdep init

rosdep update

echo "" >> ${HOME}/.bashrc
echo "# Set ROS Environment Variables:" >> ${HOME}/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ${HOME}/.bashrc
source /opt/ros/noetic/setup.bash
source ${HOME}/.bashrc

sudo apt-get --yes install python3-rosinstall

mkdir -p ${HOME}/catkin_ws/src
cd ${HOME}/catkin_ws
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc
source ${HOME}/catkin_ws/devel/setup.bash
source ${HOME}/.bashrc

sudo apt-get clean
sudo apt-get --yes install ros-noetic-rosbridge-suite

# ---------------------------------------------------
# Turtlebot3
# https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
echo ">>> Installing Turtlebot3 packages..."
sudo apt-get --yes install ros-noetic-joy ros-noetic-teleop-twist-joy
sudo apt-get --yes install ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc
sudo apt-get --yes install ros-noetic-rgbd-launch ros-noetic-rosserial-arduino
sudo apt-get --yes install ros-noetic-rosserial-python ros-noetic-rosserial-client
sudo apt-get --yes install ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server
sudo apt-get --yes install ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro
sudo apt-get --yes install ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz
sudo apt-get --yes install ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

echo ">>> Installing Turtlebot3..."
sudo apt --yes install ros-noetic-dynamixel-sdk
sudo apt --yes install ros-noetic-turtlebot3-msgs
sudo apt --yes install ros-noetic-turtlebot3

echo ">>> Installing Turtlebot3 Simulation Package..."
# https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make

echo "export TURTLEBOT3_MODEL=waffle_pi" >> ${HOME}/.bashrc

# Test
# export TURTLEBOT3_MODEL=burger
# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

# ---------------------------------------------------
# openCV
# https://www.pyimagesearch.com/2018/09/19/pip-install-opencv/

sudo apt-get update
sudo apt-get upgrade

sudo apt-get --yes install python3-dev
sudo apt-get --yes install libgl1-mesa-glx

sudo pip uninstall opencv-contrib-python && sudo pip uninstall opencv-python
sudo pip3 install opencv-contrib-python


# Other Applications
echo ">>> Installing Useful Applications..."
sudo apt-get --yes install filezilla
sudo apt-get --yes install geany
sudo apt-get --yes install gimp
sudo apt-get --yes install meld
sudo apt-get --yes install kazam
sudo apt-get --yes install retext
sudo apt-get --yes install chromium-browser


#sudo apt install software-properties-common apt-transport-https wget
#wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
#sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
#sudo apt install code

# ---------------------------------------------------
# dronekit
echo ">>> NOT installing Dronekit..."
# pip install dronekit
# pip install dronekit-sitl
# pip show dronekit
# pip show dronekit-sitl

# ---------------------------------------------------
# Python Stuff
pip install --upgrade ipython
pip install --upgrade numpy scipy matplotlib scikit-learn pandas pillow
pip install --upgrade jupyter
pip install --upgrade veroviz
pip install --upgrade pygame
pip install --upgrade pyserial
pip install --upgrade xbee
pip install --upgrade pyglet
pip install --upgrade netifaces
pip install --upgrade geocoder
pip install --upgrade geopy
pip install shapely

# sudo reboot

# MAVproxy
echo ">>> Installing MAVproxy..."
sudo apt-get --yes install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame

pip3 install PyYAML mavproxy --user

echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc


# MAVSDK
echo ">>> Installing MAVSDK..."
# pip3 install mavsdk
pip3 install mavsdk==0.20.0



# ---------------------------------------------------
# Remove Mode Manager
echo ">>> Removing Mode Manager..."
sudo apt-get --yes remove modemmanager 

# Need this for `ifconfig`
sudo apt --yes install net-tools

# ---------------------------------------------------

# ----------------------------------------------------
# Husky
# https://www.clearpathrobotics.com/assets/guides/noetic/ros/Drive%20a%20Husky.html
sudo apt-get --yes install ros-noetic-husky-desktop
sudo apt-get --yes install ros-noetic-husky-simulator
sudo apt-get --yes install ros-noetic-husky-navigation
# Test:
# roslaunch husky_gazebo empty_world.launch
# ----------------------------------------------------


echo ""
echo ""


# PX4 SITL (https://github.com/optimatorlab/soar_rover/wiki/Installation---PX4-SITL)
# echo ">>> Need to install PX4 SITL manually."

# Make sure this is in `~/.bashrc`
echo ">>> Make sure the following lines are in ~/.bashrc:"
echo "source /opt/ros/noetic/setup.bash"
echo "source ~/catkin_ws/devel/setup.bash --extend"
echo "source /usr/share/gazebo/setup.bash --extend"


# Install QGroundControl
# echo ">>> Need to install QGroundControl manually"


# ---------------------
# Coex Clover
# https://clover.coex.tech/en/simulation_native.html

# Prelims.  Already done
sudo apt --yes install build-essential git python3-pip python3-rosdep

source ~/.bashrc


# Clone Clover repos:
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/CopterExpress/clover
git clone --depth 1 https://github.com/CopterExpress/ros_led
git clone --depth 1 https://github.com/ethz-asl/mav_comm

# Install all dependencies using rosdep:
sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
cd ~/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y

# Install Python dependencies:
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt

# Clone PX4 sources and make the required symlinks:
git clone --recursive --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
ln -s ~/PX4-Autopilot ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/mavlink ~/catkin_ws/src/


# PX4 comes with its own script for dependency installation. We may as well leverage it:
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh --no-nuttx

pip3 install --user toml

# Add the Clover airframe to PX4 using the command:
ln -s ~/catkin_ws/src/clover/clover_simulation/airframes/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

# Install geographiclib datasets
# mavros package requires geographiclib datasets to be present:
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# Build your workspace:
cd ~/catkin_ws
catkin_make -j2

# Test
# roslaunch clover_simulation simulator.launch

# --------------------------------------------------------------------
# Universal Robot arms
# See https://github.com/ros-industrial/universal_robot/tree/noetic 
sudo apt-get --yes install ros-noetic-universal-robots

# Test:
# roslaunch ur_gazebo ur5_bringup.launch
# 
# MoveIt! with a simulated robot Again, you can use MoveIt! to control the simulated robot.
# For setting up the MoveIt! nodes to allow motion planning run:
# roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
#
# For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
# roslaunch ur5_moveit_config moveit_rviz.launch


# Spot (and other quadrupeds)
# See https://github.com/chvmp/champ?tab=readme-ov-file
sudo apt install -y python3-rosdep
cd ~/catkin_ws/src
git clone --recursive https://github.com/chvmp/champ
git clone https://github.com/chvmp/champ_teleop
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

cd ~/catkin_ws
catkin_make
source ~/.bashrc

# Test:
# roslaunch champ_config gazebo.launch 
# See https://github.com/chvmp/champ?tab=readme-ov-file#2-quick-start for lots more options

echo "alias stopros=\"bash ~/catkin_ws/src/soar_rover/stop_ros.sh\"" >> ${HOME}/.bashrc
echo "alias stopros=\"bash ~/stop_ros.sh\"" >> ${HOME}/.bashrc

# Our simulated husky does not have lidar/camera enabled by default.
# See `/opt/ros/noetic/share/husky_description/urdf/husky.urdf.xacro` for options
# DO. NOT. EDIT. THE. ABOVE. FILE.
# Instead, add these to .bashrc:
echo "export HUSKY_LMS1XX_ENABLED=1" >> ${HOME}/.bashrc
echo "export HUSKY_LMS1XX_SECONDARY_ENABLED=1" >> ${HOME}/.bashrc
echo "export HUSKY_REALSENSE_ENABLED=1" >> ${HOME}/.bashrc
