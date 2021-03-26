# vr-teleoperation

This package is for the teleoperation of a humanoid robot using virtual reality technology. It coincides with the publication found in this repository.

**Primary Developer: Roman Kulikovskiy**

**Documentation: Alexander Tyshka**

This package was tested on:
1) Ubuntu 18.04
2) ROS Melodic
3) Pepper Robot
4) HTC Vive

# Installation

## Pre-requisites:

- Ubuntu 18.04
- ROS I (ROS Melodic) 
- ROS packages for pepper (http://wiki.ros.org/pepper) (http://wiki.ros.org/pepper/Tutorials) or Nao

## Instructions:

### Install Steam
    sudo add-apt-repository multiverse
    sudo apt install steam steam-devices libvulkan1
Steam login is 
user: LouieLabOU, password: intelligentlabs

### Install SteamVR
Plug in headset and steamVR will prompt to install, or go to market if it does not show up and install steam vr

### Install Naoqi python SDK
https://www.softbankrobotics.com/emea/en/support/pepper-naoqi-2-9/downloads-softwares/former-versions?os=45&category=98
Add to `~/.bashrc ` file:
`export PYTHONPATH=~/naoqi/pynaoqi-python2.7-2.5.7.1-linux64/lib/python2.7/site-packages:$PYTHONPATH`

### Other Dependencies
Install QiBullet 1.4.2 with `pip install --user qibullet==1.4.2`
Install with apt:
`sudo apt install ros-melodic-naoqi-bridge-msgs libglfw3-dev libglew-dev ros-melodic-octomap-msgs ros-melodic-octomap ros-melodic-rgbd-launch ros-melodic-camera-info-manager-py ros-melodic-naoqi-driver`

### VR Teleoperation
Clone this repository into the ros workspace
Run a find-and-replace for /home/alex/ros/src/vr-teleoperation/ and change to your ros workspace

Run `catkin_make` in your workspace
Run SteamVR
#### Setup Bindings
Start the RobotControllerApp node
Go to settings > Controllers > Show Old Binding UI -> RobotControllerApp
Note: the GUI is temperamental and it may require repeated presses to open the bindings for RobotControllerApp
Set up bindings:
    - For both triggers, map to the click action
    - For both gripper, map to the gripper value action
    - For pose, map both left and right hand raw positions 
 
Save and exit. The application should be working.

Using real robot:

    roscore
    roslaunch pepper_bringup pepper_full_py.launch nao_ip:=<yourRobotIP> roscore_ip:=<roscore_ip>
    rosrun pepper_vr_controller RobotControllerApp
    rosrun pepper_naoqi_py pepper_node.py --ip <yourRobotIP>

Using Simulation:

    roscore
    rosrun pepper_vr_controller RobotControllerApp
    rosrun pepper_naoqi_py pepper_virtual.py --ip <yourRobotIP>
