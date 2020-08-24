import termios, sys , tty
import hsrb_interface
import rospy

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')
tts.language = tts.ENGLISH

whole_body.move_to_go()

omni_base.go_rel(0.7, 0.0, 0.0, 100.0)
omni_base.go_rel(0.0, -1.0, 0.0, 100.0)
# omni_base.go_rel(0.0, 0.01, 0.0, 100.0)
# omni_base.go_rel(0.0, 0.01, 0.0, 100.0)

whole_body.move_to_neutral()

whole_body.move_end_effector_by_line((0, 0, 1), 0.3)
gripper.apply_force(1)
whole_body.move_end_effector_by_line((0, 0, -1), 0.3)