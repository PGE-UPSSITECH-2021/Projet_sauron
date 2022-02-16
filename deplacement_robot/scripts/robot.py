#!/usr/bin/env python
import rospkg
import rospy
from useful_robot import get_fk, pose_msg_to_homogeneous_matrix
from run_qualite import run_qualite
from run_identification import run_identification
from run_pointage import run_pointage
from deplacement_robot.msg import Identification, Qualite, Localisation
from std_msgs.msg import Bool, String
from deplacement_robot.srv import Robot_set_state, Robot_move_predef
import moveit_commander
import sys
import time

import numpy as np

class Robot:
    def __init__(self):
        self.plaque_pos = None
        self.nom_plaque = None
        self.intrinsic = None
        self.distorsion = None
        self.H = None
        self.image_global = None # TODO
        self.moveit_commander = moveit_commander.roscpp_initialize(sys.argv)
        self.res_qualite = {}

        rospack = rospkg.RosPack()
        self.step_folder = rospack.get_path("deplacement_robot") + "/plaques"

        self.pub_result = rospy.Publisher("result", Bool, queue_size=10)
        self.pub_identification = rospy.Publisher("result/identification", Identification, queue_size=10)
        self.pub_qualite = rospy.Publisher("result/qualite", Qualite, queue_size=10)
        self.pub_localisation = rospy.Publisher("result/localisation", Localisation, queue_size=10)
        
        self.pub_prod_state = rospy.Publisher("production_state", String,  queue_size=10)

        self.srv_set_robot_state = rospy.ServiceProxy("set_robot_state", Robot_set_state)

        rospy.Subscriber("/result/ok", Bool, self.result_aquitement)

        self.aquitement = False

    # Fonction de callback qui recoit l aquitement
    def result_aquitement(self, msg):
        self.aquitement = True

    # Fonction pour envoyer un message jusqu a aquitement
    def spam_result(self, pub, msg):
        self.aquitement = False
        rate = rospy.Rate(10)
        while not self.aquitement and not rospy.is_shutdown():
            print("spam")
            pub.publish(msg)

    # Fonction pour changer l etat de la production
    def set_robot_state(self, state):
        self.srv_set_robot_state(state)

    def fin_prod(self):
        group = moveit_commander.MoveGroupCommander("manipulator")

        parking = group.get_named_target_values("parking")
        keys = parking.keys()

        parking_list = []
        for k in sorted(keys):
            parking_list.append(parking[k])        

        parking_list = np.round(parking_list, 3)
        curent_state = np.round(group.get_current_joint_values(), 3)

        if np.linalg.norm(np.array(parking_list) - np.array(curent_state)) <= 0.2:
            self.set_robot_state("LIBRE INIT")
        else:
            self.set_robot_state("LIBRE NON INIT")


    def execute_initialisation(self, send_result=True):
        self.set_robot_state("INITIALISATION")

        # Service pour deplacer le robot a sa position de parking
        move_parking = rospy.ServiceProxy('move_robot_parking', Robot_move_predef)

        move_parking()

        rospack = rospkg.RosPack()
        folder_path = rospack.get_path("deplacement_robot")
        try :
            self.intrinsic = np.loadtxt(folder_path+"/saves/intrinsec")
            self.distorsion = np.loadtxt(folder_path+"/saves/distorsion")
            if send_result:
                self.fin_prod()
            return True
        except:
            if send_result:
                self.fin_prod()
            return False

    # Fonction pour lancer la phase de calibration
    def execute_calibration(self):
        self.set_robot_state("CALIBRATION")

        #TODO self.intrinsic, self.distorsion = run_calibration()
        # Only test
        self.intrinsic = np.array([ [4.78103205e+03, 0.00000000e+00, 1.20113948e+03],
                                    [0.00000000e+00, 4.77222528e+03, 1.14533714e+03],
                                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.distorsion = np.array([[ 1.55284357e-01, -3.07067931e+00,  5.16274059e-03, -4.78075223e-03, 1.80663250e+01]])

        rospack = rospkg.RosPack()
        folder_path = rospack.get_path("deplacement_robot")
        np.savetxt(folder_path+"/saves/intrinsec", self.intrinsic)
        np.savetxt(folder_path+"/saves/distorsion", self.distorsion)

        self.fin_prod()

    # Fonction pour lancer la phase de localisation
    def execute_localisation(self, nom_plaque, send_result=True):
        # Reset des resultat de la qualite
        self.res_qualite = {}

        self.nom_plaque = nom_plaque

        #TODO : msg,self.H = run_localisation()
        msg = Localisation()
        msg.x = float(0.55)
        msg.y = float(0.24)
        msg.z = float(0.005)
        msg.a = float(0)
        msg.b = float(0)
        msg.g = float(0)
        self.plaque_pos = np.array([[1,0,0,0.55],
                                    [0,1,0,0.24],
                                    [0,0,1,0.005],
                                    [0,0,0,1]])

        time.sleep(1)

        if send_result:
            self.pub_result.publish(True)
            self.spam_result(self.pub_localisation, msg)

        return True

    # Fonction pour lancer la phase d identication
    def execute_identification(self, nom_plaque, diametres, send_result=True):
        if self.intrinsic is None or self.distorsion is None:
            self.execute_initialisation(send_result=False)
        if self.nom_plaque != nom_plaque or self.plaque_pos is None:
            self.execute_localisation(nom_plaque, send_result=False)

        # Reset des resultat de la qualite
        self.res_qualite = {}

        msg,_ = run_identification(self.plaque_pos, nom_plaque, self.step_folder, diametres, pub=self.pub_prod_state) #TODO get image global

        if send_result:
            self.pub_result.publish(True)
            self.spam_result(self.pub_identification, msg)

        return True

    # Fonction pour lancer la phase de qualites
    def execute_qualite(self, nom_plaque, diametres, send_result=True):
        if self.nom_plaque != nom_plaque or self.plaque_pos is None:
            self.execute_localisation(nom_plaque, send_result=False)
            self.execute_identification(nom_plaque, diametres, send_result=False)
        
        msg,res = run_qualite(self.plaque_pos, nom_plaque, self.step_folder, diametres=diametres, pub=self.pub_prod_state)

        if send_result:
            self.pub_result.publish(True)
            self.spam_result(self.pub_qualite, msg)

        for k in res:
            self.res_qualite[k] = res[k]

        return True

    # Fonction pour lancer la phase de qualites
    def execute_pointage(self, nom_plaque, diametres):
        if self.nom_plaque != nom_plaque or self.plaque_pos is None:
            self.execute_localisation(nom_plaque, send_result=False)
            self.execute_identification(nom_plaque, diametres, send_result=False)
            self.execute_qualite(nom_plaque, diametres, send_result=False)

        diam_non_qual = []
        for d in diametres:
            if not d in self.res_qualite:
                diam_non_qual.append(d)
        if not diam_non_qual == []:
            self.execute_qualite(nom_plaque, diam_non_qual, send_result=False)

        run_pointage(self.res_qualite, diametres)

        self.pub_result.publish(True)

        return True