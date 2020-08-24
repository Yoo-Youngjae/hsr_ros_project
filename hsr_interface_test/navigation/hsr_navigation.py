import rospy
import sys
import actionlib

from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped

class HSR_Navigation:
    def __init__(self):
        self.nav_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        print('Navigator loaded')

    def nav_goal(self, x, y, z, w):

        base_goal = MoveBaseGoal()
        target_pose = PoseStamped()
        header = Header()
        header.frame_id = 'map'

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        target_pose.header = header
        target_pose.pose = pose
        base_goal.target_pose = target_pose

        return base_goal

    def navigation(self, x, y, z, w):
        self.nav_client.wait_for_server()
        goal = self.nav_goal(x, y, z, w)
        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result()
        nav_result = self.nav_client.get_result()

#####################################################################

import math

def main():

    rospy.init_node('hsrb_navigation')
    start = (-1.618, 1.502, -1.853, 0.5)
    end  = (0,0,0,0)

    navigator = HSR_Navigation()
    pose_list = [[1.21, 0.57, 0.81, 0], [-1.787, 0.193, 2.81, 0], [-1.495, 1.721, -2.324, 0]]
    for idx, pose in enumerate(pose_list):
        navigator.navigation(pose[0], pose[1], pose[2], pose[3])
        print(idx)

if __name__ == "__main__":
    sys.exit(main())
