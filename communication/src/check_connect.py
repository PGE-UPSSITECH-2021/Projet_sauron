#!/usr/bin/env python
# coding: utf-8


import rospy
from variables_cognex import Variables
from std_msgs.msg import Bool
import telnetlib
from ftplib import FTP

global variables
variables = Variables()


##############################################
###########       CONNEXION       ############
##############################################

def check_connexion():
    try:
        # Creation of the Telnet connexion
        variables.tn = telnetlib.Telnet(variables.ip, port=23, timeout=10)
        # Creation of the FTP connexion
        variables.ftp = FTP(variables.ip)
        variables.ftp.login(variables.user)
        variables.pub_ok.publish(True)
    except:
        variables.pub_ok.publish(False)

    variables.rate.sleep()



rospy.init_node('check_camera', anonymous=False)
variables.rate = rospy.Rate(5)
variables.pub_ok = rospy.Publisher("camera/camera_ok", Bool, queue_size=10)


while(not rospy.is_shutdown()):
    check_connexion()