import qi
import rospy

from naoqi_bridge_msgs.msg import  JointAnglesWithSpeed

class PepperMotionControll:
    def __init__(self, session):
        self.session = session
        self.motion_service = session.service("ALMotion")
        self.autonomous_service = session.service("ALAutonomousLife")
        self.motion_service.setStiffnesses("Body", 1.0)
        self.autonomous_service.setState("disabled")
        self.motion_service.wakeUp()
        self.sub = rospy.Subscriber('/joint_angles', JointAnglesWithSpeed, self.callback) 

    def callback(self, msg):
        self.motion_service.setAngles(msg.joint_names, msg.joint_angles, msg.speed)