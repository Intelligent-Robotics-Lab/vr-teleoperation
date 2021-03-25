#!/usr/bin/env python
# coding: utf-8

import rospy
import pybullet_data
from qibullet import PepperVirtual
from qibullet import PepperRosWrapper
from qibullet import SimulationManager
from qibullet.camera import *


if __name__ == "__main__":
    simulation_manager = SimulationManager()

    client = simulation_manager.launchSimulation(gui=True)

    wrap = PepperRosWrapper()
    camera_id = PepperVirtual.ID_CAMERA_TOP
    robot = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    wrap.launchWrapper(robot, "/naoqi_driver")

    handle = robot.subscribeCamera(camera_id, resolution=Camera.K_VGA)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        wrap.stopWrapper()
        simulation_manager.stopSimulation(client)