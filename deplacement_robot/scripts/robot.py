#!/usr/bin/env python
import rospkg
import rospy
from run_qualite import run_qualite
from run_identification import run_identification
from deplacement_robot.msg import Identification, Qualite, Localisation
from std_msgs.msg import Bool

import numpy as np

class Robot:
    def __init__(self):
        self.plaque_pos = None
        self.nom_plaque = None
        self.param_int = None

        rospack = rospkg.RosPack()
        self.step_folder = rospack.get_path("deplacement_robot") + "/plaques"

        self.pub_result = rospy.Publisher("result/", Bool, queue_size=10)
        self.pub_identification = rospy.Publisher("result/indentification", Identification, queue_size=10)
        self.pub_qualite = rospy.Publisher("result/qualite", Qualite, queue_size=10)
        self.pub_localisation = rospy.Publisher("result/localisation", Localisation, queue_size=10)

    def execute_calibration(self):
        # TODO : self.param_int = run_calibration

        return True

    def execute_localisation(self, nom_plaque):
        self.nom_plaque = nom_plaque

        #TODO : msg,H = run_localisation
        msg = Localisation()
        msg.x = 0.55
        msg.y = 0.24
        msg.z = 0.005
        msg.a = 0
        msg.b = 0
        msg.g = 0
        self.plaque_pos = np.array([[1,0,0,0.55],
                                    [0,1,0,0.24],
                                    [0,0,1,0.005],
                                    [0,0,0,1]])

        self.pub_result.publish(True)
        self.pub_localisation.publish(msg)

        return True

    def execute_identification(self, nom_plaque, diametres):
        if self.plaque_pos is None:
            return False
        '''if not self.param_int:
            return False'''
        if self.nom_plaque != nom_plaque:
            self.execute_localisation(nom_plaque)

        msg = run_identification(self.plaque_pos, nom_plaque, self.step_folder)

        self.pub_result.publish(True)
        self.pub_identification.publish(msg)

        return True


    def execute_qualite(self, nom_plaque, diametres):
        if self.plaque_pos is None:
            return False
        if self.nom_plaque != nom_plaque:
            self.execute_localisation(nom_plaque)
            self.execute_identification(nom_plaque, diametres)
        
        msg = run_qualite(self.plaque_pos, nom_plaque, self.step_folder, diametres=diametres)

        self.pub_result.publish(True)

        self.pub_qualite.publish(msg)

        return True





