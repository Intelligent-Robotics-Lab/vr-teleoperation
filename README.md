# vr-teleoperation

This package is for the teleoperation of a humanoid robot using virtual reality technology. It coincides with the publication found in this repository.

# Contributors

Primary Developer: Roman Kulikovskiy

Documentation: Alexander Tyshka

# This package was tested on:
1) Ubuntu 18.04
2) ROS Melodic
3) Pepper Robot
4) HTC Vive

# Installation

## Pre-requisites:

- Ubuntu 18.04
- ROS I (ROS Melodic) 
- ROS packages for pepper (http://wiki.ros.org/pepper) (http://wiki.ros.org/pepper/Tutorials) or Nao
    - https://github.com/ahornung/humanoid_msgs
    - https://github.com/ros-naoqi/naoqi_bridge
    - https://github.com/ros-naoqi/nao_robot
    - https://github.com/ros-naoqi/pepper_robot

## Instructions:

### Install Steam
    sudo add-apt-repository multiverse
    sudo apt install steam steam-devices libvulkan1
Steam login is 
user: LouieLabOU, password: intelligentlabs

### Install SteamVR
Plug in headset and steamVR should prompt to install. If for some reason it does not do so, manually install by opening steam, going to store, and searching for and installing the steamvr app.

### Install Naoqi python SDK
Download the compressed sdk here, and unzip to a folder/directory of your choice (the following example uses a folder called naoqi in the home directory)
https://community-static.aldebaran.com/resources/2.5.10/Python%20SDK/pynaoqi-python2.7-2.5.7.1-linux64.tar.gz
Add to `~/.bashrc ` file:
`export PYTHONPATH=~/naoqi/pynaoqi-python2.7-2.5.7.1-linux64/lib/python2.7/site-packages:$PYTHONPATH`
If using another install path, instead use `export PYTHONPATH=<YOUR DIRECTORY>/pynaoqi-python2.7-2.5.7.1-linux64/lib/python2.7/site-packages:$PYTHONPATH`

### Other Dependencies
Install QiBullet 1.4.2 with `pip install --user qibullet==1.4.2`
Install with apt:
`sudo apt install ros-melodic-naoqi-bridge-msgs libglfw3-dev libglew-dev ros-melodic-octomap-msgs ros-melodic-octomap ros-melodic-rgbd-launch ros-melodic-camera-info-manager-py ros-melodic-naoqi-driver`

### VR Teleoperation
Clone this repository into the ros workspace
Run a find-and-replace for /home/alex/ros/src/vr-teleoperation/ and change to your ros workspace
This can be done in the IDE of your choice (recommended for verification) or by running the following command in the root of this repository:
```find ./ -type f -exec sed -i -e 's|/home/alex/ros/src/vr-teleoperation|'`pwd`'|g' {} \;```

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
 
![Screenshot from 2021-04-01 15-22-07](https://user-images.githubusercontent.com/19317207/113343704-2aa90300-92fe-11eb-9ee4-00ff5f031f8b.png)
![Screenshot from 2021-04-01 15-22-34](https://user-images.githubusercontent.com/19317207/113343776-401e2d00-92fe-11eb-8ad5-9035e34a3f2d.png)

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

For IP address, on a real robot pepper/nao should speak their IP address after turning on and connecting to the network. For a simulation, 0.0.0.0 can be used for the ip address.
