#!/usr/bin/python

import alsaaudio, time, audioop
import rospy
import struct
from naoqi_bridge_msgs.msg import AudioBuffer

class AudioPlayback():
    def __init__(self, name):
        rospy.init_node(name)
        print alsaaudio.pcms(alsaaudio.PCM_PLAYBACK)
        self.outp = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK,alsaaudio.PCM_NORMAL)

        # Set attributes: Mono, 8000 Hz, 16 bit little endian samples
        self.outp.setchannels(4)
        self.outp.setrate(16000)
        self.outp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self.outp.setperiodsize(1365)
        self.sub_mic = rospy.Subscriber("naoqi_microphone/audio_raw", AudioBuffer, self.recv_audio, queue_size=1000)
        self.output_buffer = []
        self.timer = rospy.Timer(rospy.Duration(0.00001), self.send_audio)

    def recv_audio(self, msg):
	data = msg.data
        #print struct.pack('%uh' % len(msg.data), *msg.data)
	self.outp.write(struct.pack('%uh' % len(data), *data))
        #self.output_buffer.append(msg.data)
        # 
    
    def send_audio(self, event):
        if (len(self.output_buffer) > 0):
            
            data = self.output_buffer.pop()
            print(len(data))
            self.outp.write(struct.pack('%uh' % len(data), *data))
        # else:
        #     data = [0, 0, 0, 0]
        #     self.outp.write(struct.pack('%uh' % len(data), *data))

if __name__ == "__main__":
    playback = AudioPlayback('audio_playback')
    rospy.spin()
