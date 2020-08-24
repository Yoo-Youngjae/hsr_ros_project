import pyaudio
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import atexit
import time

rospy.init_node('mic_receiver')
pub_mic = rospy.Publisher('/snu/microphone_send', String, queue_size=10)

mat = Float64MultiArray()


CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print("* recording")
frame = []

while True:
    data = stream.read(CHUNK)
    # mat.data = np.fromstring(data, dtype=np.float32).tolist()
    pub_mic.publish(String(data))
    # time.sleep(0.02)

atexit.register(stream.stop_stream)
atexit.register(stream.close)
atexit.register(p.terminate)
