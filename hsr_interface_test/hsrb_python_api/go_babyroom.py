import hsrb_interface
import rospy

rospy.init_node('go_babyroom')

robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')

whole_body.move_to_go()
print(1)
omni_base.go_abs(1.53, 0.3, 1.5, 100)
print(2)
omni_base.go_abs(1.48, 1.95, 1.7, 100)
print(3)
omni_base.go_abs(1.81, 2.51, 0.81, 100)
print(4)
omni_base.go_abs(1.81, 2.51, 0.81, 100)
print(5)
omni_base.go_abs(2.34, 3.30, 1.40, 100)
print(6)
omni_base.go_abs(2.69, 3.81, 0.13, 100)
