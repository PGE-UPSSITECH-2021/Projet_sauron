#!/usr/bin/env python

import rospy
from industrial_msgs.msgs import RobotStatus 

global status
status = "NOK"

def callback_robot_status(msg):
    if msg.e_stopped or msg.in_error :
        status = "NOK"
    else :
        status = "OK"
    

pub_secu = rospy.Publisher("securite_state", String, queue_size=10)
rospy.init_node('move_robot_server', anonymous=True)
rospy.Subscriber("/robot_status", RobotStatus, callback_robot_status)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub_secu.publish(status)
    rate.sleep()