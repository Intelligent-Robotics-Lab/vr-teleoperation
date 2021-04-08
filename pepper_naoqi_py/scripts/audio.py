#!/usr/bin/python

import alsaaudio, time, audioop
import rospy
from naoqi_driver.naoqi_node import NaoqiNode

class NaoAudio(NaoqiNode):
    def __init__(self, name):
        NaoqiNode.__init__(self, name)

        self.microVersion = 0

        # Create a proxy to ALAudioDevice
        try:
            self.audioProxy = self.get_proxy("ALAudioDevice")
        except Exception, e:
            rospy.logerr("Error when creating proxy on ALAudioDevice:")
            rospy.logerr(str(e))
            exit(1)

        try:
            self.robotProxy = self.get_proxy("ALRobotModel")
            self.microVersion = self.robotProxy._getMicrophoneConfig()
        except Exception, e:
            rospy.logwarn("Could not retrieve microphone version:")
            rospy.logwarn(str(e))
            rospy.logwarn("Microphone channel map might not be accurate.")

        self.inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE,alsaaudio.PCM_NONBLOCK)

        # Set attributes: Mono, 8000 Hz, 16 bit little endian samples
        self.inp.setchannels(2)
        self.inp.setrate(22050)
        self.inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self.inp.setperiodsize(1000)

    def run(self):
        r=rospy.Rate(1000)
        self.audioProxy.setOutputVolume(100)
        while self.is_looping():
            l,data = self.inp.read()
            if l:
                # Return the maximum of the absolute value of all samples in a fragment.
                self.audioProxy.sendRemoteBufferToOutput(len(data)/4, data)
            r.sleep()

if __name__ == "__main__":
    NaoSpeaker = NaoAudio('NaoSpeaker')
    NaoSpeaker.start()
    rospy.spin()
