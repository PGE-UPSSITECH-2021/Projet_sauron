#!/usr/bin/env python
# coding: utf-8


import rospy
import time
from variables_cognex import Variables
from std_msgs.msg import Bool
import glob
from time import sleep

global variables
variables = Variables()


##############################################
###########       CONNEXION       ############
##############################################

def check_connexion():
        try:
            variables.pub_ok.publish(True)
            variables.rate.sleep()
        except:
            variables.pub_ok.publish(False)
            variables.rate.sleep()



rospy.init_node('check_camera', anonymous=False)
variables.rate = rospy.Rate(5)
variables.pub_ok = rospy.Publisher("camera/camera_ok", Bool, queue_size=10)


while(True):
    
    check_connexion(None)


