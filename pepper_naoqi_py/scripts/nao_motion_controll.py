import qi
import rospy

from trajectory_msgs.msg._JointTrajectory import JointTrajectory
from naoqi_bridge_msgs.msg import  JointAnglesWithSpeed

class NaoMotionControll:
    def __init__(self, session):
        self.session = session
        self.motion_service = session.service("ALMotion")
        # self.motion_service.setStiffness("Body", 1.0)

        self.sub = rospy.Subscriber('/joint_angles', JointAnglesWithSpeed, self.callback) 

    def callback(self, msg):
        self.motion_service.SetAngles(msg.joint_names, msg.joint_angles, msg.speed)
    
    def __del__(self):
        self.motion_service.Destroy()