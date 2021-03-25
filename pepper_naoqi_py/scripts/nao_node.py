#!/usr/bin/env python

import argparse
import sys
import time

import qi

import rospy

from nao_motion_controll import NaoMotionControll

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number.")
    args = parser.parse_args()
    session = qi.Session()


    try:
        rospy.init_node("nao_naoqi_py_node", anonymous=True)
            
        session.connect("tcp://" + args.ip + ":" + str(args.port))
        NaoMotionControll(session)
    except RuntimeError:
        rospy.logerr("Can't connect to Naoqi at ip \"%s\" on port %d.\n"
                     "Please check your script arguments. Run with -h option for help.", args.ip, args.port)
        sys.exit(1)
        

    rospy.spin()