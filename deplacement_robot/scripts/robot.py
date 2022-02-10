#!/usr/bin/env python
import rospkg
import rospy
from useful_robot import get_fk, pose_msg_to_homogeneous_matrix
from run_qualite import run_qualite
from run_identification import run_identification
from deplacement_robot.msg import Identification, Qualite, Localisation
from std_msgs.msg import Bool, String
from deplacement_robot.srv import Robot_set_state
import moveit_commander
import sys

import numpy as np

class Robot:
    def __init__(self):
        self.plaque_pos = None
        self.nom_plaque = None
        self.intrinsic = None
        self.H = None
        self.image_global = None # TODO
        self.moveit_commander = moveit_commander.roscpp_initialize(sys.argv)

        rospack = rospkg.RosPack()
        self.step_folder = rospack.get_path("deplacement_robot") + "/plaques"

        self.pub_result = rospy.Publisher("result/", Bool, queue_size=20)
        self.pub_identification = rospy.Publisher("result/indentification", Identification, queue_size=20)
        self.pub_qualite = rospy.Publisher("result/qualite", Qualite, queue_size=20)
        self.pub_localisation = rospy.Publisher("result/localisation", Localisation, queue_size=20)

        self.srv_set_robot_state = rospy.ServiceProxy("set_robot_state", Robot_set_state)

        rospy.Subscriber("/result/ok", Bool, self.result_aquitement)

        self.aquitement = False

    # Fonction de callback qui recoit l aquitement
    def result_aquitement(self, msg):
        self.aquitement = True

    # Fonction pour envoyer un message jusqu a aquitement
    def spam_result(self, pub, msg):
        self.aquitement = False
        rate = rospy.Rate(20)
        while not self.aquitement and not rospy.is_shutdown():
            pub.publish(msg)

    # Fonction pour changer l etat de la production
    def set_robot_state(self, state):
        self.serv_set_robot_state(state)

    def fin_prod(self):
        group = moveit_commander.MoveGroupCommander("manipulator")

        parcking = group.get_named_target_values("parcking")
        keys = parcking.keys()

        parcking_list = []
        for k in sorted(keys):
            parcking.append(d[k])        

        parcking_list = np.round(parcking_list, 3)
        curent_state = np.round(groupget_current_joint_values(), 3)

        if (parcking_list == curent_state).all():
            self.set_robot_state("LIBRE INIT")
        else:
            self.set_robot_state("LIBRE NON INIT")



    # Fonction pour lancer la phase de calibration
    def execute_calibration(self):
        # TODO : self.intrinsic = run_calibration()

        return True

    # Fonction pour lancer la phase de localisation
    def execute_localisation(self, nom_plaque, send_result=True):
        self.nom_plaque = nom_plaque

        #TODO : msg,self.H = run_localisation()
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

        if send_result:
            self.pub_result.publish(True)
            self.spam_result(self.pub_localisation, msg)

        return True

    # Fonction pour lancer la phase d identication
    def execute_identification(self, nom_plaque, diametres, send_result=True):
        if self.plaque_pos is None:
            return False
        '''if not self.param_int:
            return False'''
        if self.nom_plaque != nom_plaque:
            self.execute_localisation(nom_plaque, send_result=False)

        msg = run_identification(self.plaque_pos, nom_plaque, self.step_folder)

        if send_result:
            self.pub_result.publish(True)
            self.spam_result(self.pub_identification, msg)

        return True

    # Fonction pour lancer la phase de qualites
    def execute_qualite(self, nom_plaque, diametres):
        if self.plaque_pos is None:
            return False
        if self.nom_plaque != nom_plaque:
            self.execute_localisation(nom_plaque, send_result=False)
            self.execute_identification(nom_plaque, diametres, send_result=False)
        
        msg = run_qualite(self.plaque_pos, nom_plaque, self.step_folder, diametres=diametres)

        self.pub_result.publish(True)
        self.spam_result(self.pub_qualite, msg)

        return True





