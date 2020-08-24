#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
def arm_handle_service(node_name, joint_positions, joint_velocities):


    # initialize ROS publisher
    pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                          trajectory_msgs.msg.JointTrajectory, queue_size=10)

    # wait to establish connection between the controller
    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # make sure the controller is running
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    list_controllers = (
        rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                           controller_manager_msgs.srv.ListControllers))
    running = False
    while running is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'arm_trajectory_controller' and c.state == 'running':
                running = True

    # fill ROS message
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                        "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    # in p, there is 5 properties 1. positions, 2. velocities, 3. acceleratioins, 4. effort, 5. time_from_start
    p.positions = joint_positions
    p.velocities = joint_velocities
    p.time_from_start = rospy.Time(3)
    traj.points = [p]

    # publish ROS message
    pub.publish(traj)

if __name__ == "__main__":
    node_name = 'arm_test'
    joint_name = ["arm_lift_joint", "arm_flex_joint",
                  "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    joint_positions = [0, -1, 0, -0.5, 0]
    joint_velocities = [0, 0, 0, 0, 0]
    rospy.init_node(node_name)
    arm_handle_service(node_name, joint_positions, joint_velocities)