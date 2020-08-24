import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time



rospy.init_node('hand_camera_receiver')
pub_hand_rgb = rospy.Publisher('/snu/hand_camera_image_raw', Image, queue_size=2)
bridge = CvBridge()

capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
index = 0
while True:
    index += 1
    try:
        ret, frame = capture.read()
        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        img_msg.width = 640
        img_msg.height = 480
        pub_hand_rgb.publish(img_msg)
        if index % 40  == 0:
            print('connect continue')
            index = 0
        # time.sleep(0.02)
    except :
        try:
            capture = cv2.VideoCapture(2)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if index % 40 == 0:
                print('connect 2')
        except:
            try:
                capture = cv2.VideoCapture(0)
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                if index % 40 == 0:
                    print('connect 0')
            except Exception as e:
                print('connect error')
                print(e)