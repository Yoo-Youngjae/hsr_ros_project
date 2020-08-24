import rospy
from hsrb_interface import Robot

import time
import math
import os
import sys
import actionlib


from actionlib_msgs.msg import GoalStatus
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)

from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)

from sensor_msgs.msg import JointState
import trajectory_msgs.msg
import controller_manager_msgs.srv
from hsrb_interface import geometry


_CONNECTION_TIMEOUT = 10.0

class MoveController(object):
    def __init__(self):
        self.pub = rospy.Publisher(
            '/hsrb/omni_base_controller/command',
            trajectory_msgs.msg.JointTrajectory, queue_size=10)
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'omni_base_controller' and c.state == 'running':
                    running = True
    def move(self, pose):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [pose[0], pose[1], pose[2]]
        p.velocities = [0, 0, 0]
        p.time_from_start = rospy.Time(15)
        traj.points = [p]

        # publish ROS message
        self.pub.publish(traj)

class JointController(object):
    """Control arm and gripper"""

    def __init__(self):
        joint_control_service = '/safe_pose_changer/change_joint'
        grasp_action = '/hsrb/gripper_controller/grasp'
        self._joint_control_client = rospy.ServiceProxy(
            joint_control_service, SafeJointChange)

        self._gripper_control_client = actionlib.SimpleActionClient(
            grasp_action, GripperApplyEffortAction)

        # Wait for connection
        try:
            self._joint_control_client.wait_for_service(
                timeout=_CONNECTION_TIMEOUT)
            if not self._gripper_control_client.wait_for_server(rospy.Duration(
                    _CONNECTION_TIMEOUT)):
                raise Exception(grasp_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def move_to_joint_positions(self, goal_joint_states):
        """Joint position control"""
        try:
            req = SafeJointChangeRequest(goal_joint_states)
            res = self._joint_control_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        return res.success

    def grasp(self, effort):
        """Gripper torque control"""
        goal = GripperApplyEffortGoal()
        goal.effort = effort

        # Send message to the action server
        if (self._gripper_control_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False


front_of_kitchen_table =   [-0.40047664762883317, -2.283359669110555, 1.5953280785572044]
front_of_washing_stand = [0.1386376617987047, -2.8358232473365064, -1.5572969007060904]
front_of_trash_box = [-2.159019715190639, -1.5350036228128476, 2.0732245784567773]
if __name__ == '__main__':
    rospy.init_node('hsrb_wash_stand_demo')
    robot = Robot()
    whole_body = robot.try_get('whole_body')
    omni_base = robot.try_get('omni_base')
    tts = robot.try_get('default_tts')
    tts.language = tts.ENGLISH
    
    whole_body.move_to_go()

    omni_base.go_abs(front_of_kitchen_table[0], front_of_kitchen_table[1], front_of_kitchen_table[2], 100.0)
    print('reach kitchen table')
    tts.say("Hi. My name is HSR. I'll move the dishes")

    joint_controller = JointController()
    move_controller = MoveController()

    rospy.sleep(2)

    initial_position = JointState()
    initial_position.name.extend(['arm_lift_joint', 'arm_flex_joint',
                                  'arm_roll_joint', 'wrist_flex_joint',
                                  'wrist_roll_joint', 'head_pan_joint',
                                  'head_tilt_joint', 'hand_motor_joint'])
    initial_position.position.extend([0.3, 0.0, 0.0, -1.57,
                                      0.0, 0.0, 0.0, 1.2])
    joint_controller.move_to_joint_positions(initial_position)
    print('stand up')
    for i in range(2):
        tts.say("Please give me a dish")
        rospy.sleep(3.5)
        joint_controller.grasp(-0.1)
        print('catch dish')

        omni_base.go_abs(front_of_washing_stand[0], front_of_washing_stand[1], front_of_washing_stand[2], 100.0)
        print('reach washing stand')
        # move_controller.move(front_of_washing_stand)


        ######## drop object pose ##########
        drop_position = JointState()
        drop_position.name.extend(['arm_lift_joint', 'arm_flex_joint',
                                      'arm_roll_joint', 'wrist_flex_joint',
                                      'wrist_roll_joint', 'head_pan_joint',
                                      'head_tilt_joint'])
        drop_position.position.extend([0.6, -1.3, 0.0, -1.0,
                                          0.0, 0.0, 0.0])
        joint_controller.move_to_joint_positions(drop_position)


        drop_hand = JointState()
        drop_hand.name.extend(['hand_motor_joint'])
        drop_hand.position.extend([1.2])
        joint_controller.move_to_joint_positions(drop_hand)


        drop_position = JointState()
        drop_position.name.extend(['arm_lift_joint', 'arm_flex_joint','wrist_flex_joint'])
        drop_position.position.extend([0.3, 0.0, -1.57])
        joint_controller.move_to_joint_positions(drop_position)
        print('end one job')
        #########################################
        omni_base.go_abs(front_of_kitchen_table[0], front_of_kitchen_table[1], front_of_kitchen_table[2], 100.0)
        print('arrive at kitchen table')
        # move_controller.move(front_of_kitchen_table)

    whole_body.move_to_go()
    tts.say("My work is done. Bye bye")




