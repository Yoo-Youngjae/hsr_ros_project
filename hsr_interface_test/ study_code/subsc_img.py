#!/usr/bin/env python
import argparse
import numpy as np
from PIL import Image
# import zbarlight
# TODO: qr libraries: qrcode, qrtool, zbarlight

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as rosImage
from sensor_msgs.msg import CameraInfo as cameraImage
from std_msgs.msg import String


cvbridge = None
p = None


def qr_decode(img):
    """ Decodes qr code in the image
    Args: img: numpy image
    Returns: string on success, None on failure
    """
    pil_img = Image.fromarray(img, 'RGB')
    codes = zbarlight.scan_codes(['qrcode'], pil_img)
    # TODO: from https://pypi.org/project/zbarlight/
    # TODO: below might not be necessary, though
    # new_image = zbarlight.copy_image_on_background(pil_img, color=zbarlight.WHITE)
    if codes:
        print(codes)
    if not codes:
        return None
    return codes[0]  # output is a list


def image_callback(imgmsg):
    global cvbridge, p
    img = cvbridge.imgmsg_to_cv2(imgmsg, 'rgb8')  # for PIL Image
    pil_img = Image.fromarray(img, 'RGB')
    pil_img.show()
    # result = qr_decode(img)

    # if not result:
    #     # do not publish
    #     return


def main(topic):
    global cvbridge
    rospy.init_node('img_reader')
    rospy.Subscriber(topic, cameraImage, image_callback)
    rospy.spin()


if __name__ == "__main__":
    topic = '/hsrb/hand_camera/camera_info'
    main(topic)
