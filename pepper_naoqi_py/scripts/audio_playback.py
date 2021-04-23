#!/usr/bin/python

import alsaaudio, time, audioop
import rospy
from naoqi_bridge_msgs.msg import AudioBuffer

class AudioPlayback():
    def __init__(self, name):
        rospy.init_node(name)

        self.outp = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK,alsaaudio.PCM_NORMAL)

        # Set attributes: Mono, 8000 Hz, 16 bit little endian samples
        self.outp.setchannels(4)
        self.outp.setrate(48000)
        self.outp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self.outp.setperiodsize(1000)
        self.sub_mic = rospy.Subscriber("audio_raw", AudioBuffer, self.recv_audio, queue_size=100)

    def recv_audio(self, msg):
        self.outp.write(msg.data)

if __name__ == "__main__":
    playback = AudioPlayback('audio_playback')
    rospy.spin()
