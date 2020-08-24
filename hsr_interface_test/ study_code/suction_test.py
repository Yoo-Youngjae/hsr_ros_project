#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Toyota Motor Corporation
"""Suction Controller Sample"""

import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
from tmc_suction.msg import (
    SuctionControlAction,
    SuctionControlGoal
)

_CONNECTION_TIMEOUT = 10.0

# Wait until pressure sensor is True
# If it is negative number, goal will be rejected
_SUCTION_TIMEOUT = rospy.Duration(20.0)


def main():
    # Create action client to control suction
    suction_action = '/hsrb/suction_control'
    suction_control_client = actionlib.SimpleActionClient(
        suction_action, SuctionControlAction)

    # Wait for connection
    try:
        if not suction_control_client.wait_for_server(
                rospy.Duration(_CONNECTION_TIMEOUT)):
            raise Exception(suction_action + ' does not exist')
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)

    # Send a goal to start suction
    rospy.loginfo('Suction will start')
    suction_on_goal = SuctionControlGoal()
    suction_on_goal.timeout = _SUCTION_TIMEOUT
    suction_on_goal.suction_on.data = True
    if (suction_control_client.send_goal_and_wait(suction_on_goal) ==
            GoalStatus.SUCCEEDED):
        rospy.loginfo('Suction succeeded. Suction will stop')

        # Send a goal to stop suction
        suction_off_goal = SuctionControlGoal()
        suction_off_goal.suction_on.data = False
        suction_control_client.send_goal_and_wait(suction_off_goal)
    else:
        rospy.loginfo('Suction failed')

if __name__ == '__main__':
    rospy.init_node('hsrb_suction_controller')
    main()