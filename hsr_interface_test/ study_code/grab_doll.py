import hsrb_interface
from base_test import base_handle_service
from arm_test import arm_handle_service
import rospy

rospy.init_node('grab_doll')

robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')

# node_name = 'arm_test'
# joint_name = ["arm_lift_joint", "arm_flex_joint",
#               "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
# joint_positions = [0, 0, 0, 0, 0]
# joint_velocities = [0, 0, 0, 0, 0]
# arm_handle_service(node_name, joint_positions, joint_velocities)

node_name = 'arm_test'
joint_name = ["arm_lift_joint", "arm_flex_joint",
              "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
joint_positions = [0, -1, 0, -0.45, 0]
joint_velocities = [0, 0, 0, 0, 0]
arm_handle_service(node_name, joint_positions, joint_velocities)


gripper.set_distance(0.15)
rospy.sleep(5)
# omni_base.go_rel(0.2, 0, 0, 100)
gripper.apply_force(1)
# omni_base.go_rel(-0.2, 0, 0, 100)
whole_body.move_to_neutral()

