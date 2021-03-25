cmake_minimum_required(VERSION 2.8.3)
project(pepper_naoqi_py)

find_package(catkin REQUIRED COMPONENTS rospy cv_bridge trajectory_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS)

catkin_install_python(PROGRAMS
   # SOURCES FOR .py FILES
   scripts/pepper_node.py
   scripts/pepper_virtual.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
