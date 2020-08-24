#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
"""Change Status LED Color Sample"""
import rospy
from std_msgs.msg import ColorRGBA

_LED_INPUT_DATA_SIZE = 256
_DEFAULT_COLOR = ColorRGBA(g=0.2, b=0.6)


def main():
    # Create Publisher to change status led color
    status_led_topic = '/hsrb/command_status_led_rgb'
    led_pub = rospy.Publisher(status_led_topic,
                              ColorRGBA, queue_size=100)

    # Wait for connection
    while led_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # 50Hz is enough frequency to do gradual color change
    rate = rospy.Rate(50)

    # Value r changes gradually (value g and b are fixed)
    color = _DEFAULT_COLOR
    for num in range(_LED_INPUT_DATA_SIZE):
        color.r = num / float(_LED_INPUT_DATA_SIZE - 1)
        led_pub.publish(color)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('hsrb_change_led_color')
    main()