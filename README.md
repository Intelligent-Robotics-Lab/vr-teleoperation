# Virtual Reality Teleoperation of a Humanoid Robot

This package is for the teleoperation of a humanoid robot using virtual reality technology. It coincides with the publication:

R. Kulikovskiy, M. Sochanski, A. Hijaz, M. Eaton, J. Korneder, and W.-Y. G. Louie, “Can Therapists Design Robot-Mediated Interventions and Teleoperate Robots Using VR to Deliver Interventions for ASD?,” IEEE International Conference on Robotics and Automation, 2021.

Pre-Print Available at: https://github.com/Intelligent-Robotics-Lab/vr-teleoperation/blob/main/IEEE_ICRA_2021.pdf

# Contributors

Primary Developer: Roman Kulikovskiy

Documentation: Alex Tyshka, Sean Dallas

# Intelligent Robotics Lab Info
For more information on the Intelligent Robotics Lab at Oakland University visit: https://www.geoffrey-louie.com/team

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
    - (put these files into your src folder)

## Instructions:

### Install Steam
    sudo add-apt-repository multiverse
    sudo apt install steam steam-devices libvulkan1
Steam login is 

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
Clone this repository into the src folder of your ros-workspace
Run a find-and-replace for /home/alex/ros/src/vr-teleoperation/ and change to your ros-workspace
This can be done in the IDE of your choice (recommended for verification) or by running the following command in the root of this repository:
```find ./ -type f -exec sed -i -e 's|/home/alex/ros/src/vr-teleoperation|'`pwd`'|g' {} \;```

Run `catkin_make` in your ros-workspace directory

Potential Error:

![Screenshot from 2021-05-28 12-12-04](https://user-images.githubusercontent.com/18145221/120015119-ef9b1700-bfb0-11eb-9a2e-e70de5a3a8cd.png)


If `catkin_make` fails you may need to download git kraken and open the vr-teleoperation repo with it. Scroll down to the bottom left and initialize the 7 submodules there. After that finishes you should be able to `catkin_make`

NOTE: May need to move the two folders from the vr-teleoperation folder into your ros-workspace/src/ folder if later when attempting to run the RobotControllerApp the program has trouble finding the shaders.


Run SteamVR
#### Setup Bindings
Before you can start the RobotControllerApp, you need to have two terminals open one in the ros-workspace and the other can be either there or a directory above it. In the latter enter the command ```roscore```. In the former you can run the command ```source devel/setup.bash``` and then run the command ```rosrun pepper_vr_controller RobotControllerApp``` to Start the RobotControllerApp node. NOTE: this command won't run until you have SteamVR open.

In the SteamVR application go to settings > Controllers > Show Old Binding UI -> RobotControllerApp
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
    rosrun pepper_naoqi_py pepper_virtual.py

For IP address, on a real robot pepper/nao should speak their IP address after turning on and connecting to the network

Audio node:
First install audio library with `apt install python-alsaaudio`
Next, run audio node with:

    rosrun pepper_naoqi_py audio.py --pip <robot-ip>
    
Camera node:

ssh into pepper and run:

    gst-launch-0.10 -v v4l2src device=/dev/video0 ! 'video/x-raw-yuv,width=640, height=480,framerate=30/1' ! ffmpegcolorspace ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.102 port=3000

On the main computer run:
roslaunch pepper_naoqi_py gstreamer.launch
    
# System Overview

VR Teleoperation consists of several modules/ros nodes: 

- The direct interface with steam is handled by the RobotControllerApp node. This uses the C++ OpenVR API to communicate with the headset. It is listening to the positions of the head and hands, and every time these update it computes joint angles. This is based on a human inverse kinematics model. This node also subscribes to camera data which is displayed on the headset.
- The pepper_bringup node is responsible for managing all of the general robot functions. Currently, the only topic in use is the camera but it would still be useful for additions such as base movement
- For controlling joint angles, we use a separate node that subscribes to the joint angles from the RobotControllerApp and outputs them to the robot via naoqi python API.
- Microphone control consists of several modules. 
    - For sending audio from the vr headset to the robot speakers, there is no speaker support in the ros driver so we use a custom node, nao_speaker. This uses the Linux alsaaudio API to get audio from the mic and sent it to the robot via Naoqi. There is a known bug with high audio latency when using camera simultaneously. Also note that this node is run through ROS but does not publish any of the data to os topics, this would need to be modified to do so if, for example, the data was logged to ros bags.
    - For listening to the audio from the robot, there is a node in naoqi_sensors_py that subscribes to the audi through naoqi and publshes it to ROS. A separete node, audio_playback.py, takes this data and outputs to the VR speakers, again using alsaaudio. This part is unaffected by the camera latency issue.
- (WIP) New camera system. Given the low resolution of the current camera implementation and impacts on audio, I'm working on a new camera approach using gstreamer. This requires ssh-ing into the pepper robot, running a gstreamer node that publishes video over a UDP socket, and running a listener on the host machine. Verified operation in gstreamer but the ros bridge is a work in progress. This is slighly more work as the ssh must be performed after any reboot, but the performance impact is significant, details here: https://groups.google.com/g/ros-sig-aldebaran/c/LhmKTxyTn1Y

