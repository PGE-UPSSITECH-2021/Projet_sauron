#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from motoman_hc10_moveit_config.msg import IHM_msg

from random import choice

def talker():
  pub = rospy.Publisher('ihm_topic', IHM_msg, queue_size=10)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(1) # 10hz
  while not rospy.is_shutdown():

    #ihm = input()
    ihm = IHM_msg()
    ihm.action = choice(['S_LOC', 'S_ID', 'S_QUAL'])
    ihm.plaque = 'tole cintree'
    ihm.diametre = '5 mm'
    ihm.confiance = '62'

    rospy.loginfo(ihm)
    pub.publish(ihm)

    rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass