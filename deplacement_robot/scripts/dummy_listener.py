#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from motoman_hc10_moveit_config.msg import IHM_msg

def extract_action(data):
    return data.action
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", extract_action(data))
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("ihm_topic", IHM_msg, callback)
    rospy.spin()
if __name__ == '__main__':
    listener()