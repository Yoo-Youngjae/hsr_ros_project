import rospy
import trajectory_msgs.msg

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

rospy.init_node('listener')

rospy.Subscriber("/hsrb/omni_base_controller/command", trajectory_msgs.msg.JointTrajectory, callback)

rospy.spin()