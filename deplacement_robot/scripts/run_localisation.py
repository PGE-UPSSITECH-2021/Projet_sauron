#!/usr/bin/env python
# coding: utf-8

import numpy as np
import rospy
import rospkg
from deplacement_robot.srv import Robot_move_predef, Move_predef
from deplacement_robot.msg import Localisation
from communication.srv import capture
from cv_bridge import CvBridge
from useful_robot import pose_msg_to_homogeneous_matrix,get_fk
from pyquaternion import Quaternion
from localisation.main import main_localisation as main_loc

class UntrustworthyLocalisationError(Exception):
    def __init__(self,message):
        self.message = message
        super(UntrustworthyLocalisationError, self).__init__(message)

class MatchingError(Exception):
    def __init__(self,message):
        self.message = message
        super(MatchingError, self).__init__(message)

def localiser(type_plaque="tole plate",model_path="",image="",M_hom_3D="",M_pass_oc="",M_intr="",coeff_distr=""):
    try:
        trans,rot,matrice_extr,bryant = main_loc()
        return trans,rot,matrice_extr,bryant
    except(UntrustworthyLocalisationError,MatchingError), e:
        return str(e)



def move_to_point(p):
    move_robot = rospy.ServiceProxy("move_predef", Move_predef)
    move_robot("localisation_"+str(p))
    print("moving to point "+str(p))


def send_results(pts):
    message = Localisation()
    message.x = pts[0]
    message.y = pts[1]
    message.z = pts[2]
    message.a = pts[3]
    message.b = pts[4]
    message.g = pts[5]

def get_image():
    #capturer l'image
    bridge=CvBridge()
    capture_image = rospy.ServiceProxy("camera/capture", capture)
    img = capture_image()
    rosimage = img.image
    cv_image = bridge.imgmsg_to_cv2(rosimage,'bgr8')
    print("shape = ",cv_image.shape)
    assert (len(cv_image.shape) == 3),"(1) probleme dimensions, image BGR ?"
    return cv_image



def run_localisation(path):
    distortion_coefs = np.array([   1.55284357e-01,
                                    -3.07067931e+00,  
                                    5.16274059e-03, 
                                    -4.78075223e-03,
                                    1.80663250e+01])


    intrinsic_mat = np.array([  [4.78103205e+03, 0.00000000e+00, 1.20113948e+03],
                                [0.00000000e+00, 4.77222528e+03, 1.14533714e+03],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
                                Mtrix_hom_3D = pose_msg_to_homogeneous_matrix(pose_get_fk)
    M_pass_oc = np.eye(4)

    for i in range(4):
        move_to_point(i+1)
        #capturer l'image
        img = get_image()
        pose_get_fk = get_fk()
        
        try:
            trans,rot,matrice_extr,bryant = localiser(type_plaque="tole plate",model_path=path,image=img,M_hom_3D=Mtrix_hom_3D,M_pass_oc=M_pass_oc,M_intr=,coeff_distr=)
            send_results([trans,rot])
            #send_results[1,2,3,4,5,6]
            break
        except TypeError:
            print("plaque non trouvée")


if __name__ == "__main__":
    rospy.init_node('test_localisation', anonymous=True)
    rospack = rospkg.RosPack()
    path = rospack.get_path("deplacement_robot/plaques")
    run_localisation(path)



