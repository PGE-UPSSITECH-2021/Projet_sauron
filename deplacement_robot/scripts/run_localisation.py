from typing import Type
import numpy as np
import rospy
from deplacement_robot.srv import Robot_move_predef
from deplacement_robot.msg import Localisation
from communication.srv import capture
from cv_bridge import CvBridge
from useful_robot import pose_msg_to_homogeneous_matrix,get_fk
from pyquaternion import Quaternion

class UntrustworthyLocalisationError(Exception):
    def __init__(self,message):
        self.message = message
        super(UntrustworthyLocalisationError, self).__init__(message)

class MatchingError(Exception):
    def __init__(self,message):
        self.message = message
        super(MatchingError, self).__init__(message)


def localiser(type_plaque,model_path,image,M_hom_3D,M_pass_oc,M_intr,coeff_distr):
    try:
        pts,matrice_extr = main_loc()
    except(UntrustworthyLocalisationError,MatchingError), e:
        return str(e)
    return pts,matrice_extr

def main_loc():
    #insert fonction de localisation
    pass

def move_to_point(p):
    move_robot = rospy.ServiceProxy('localisation_'+str(p), Robot_move_predef)
    move_robot()
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



def run_localisation():
    #bouger au points prédéfinits
    for i in range(4):
        move_to_point(i+1)
    #capturer l'image
        img = get_image()
    #catch les exceptions (retour de mauvais type de donnée)
        pose_get_fk = get_fk()
        Mtrix_hom_3D = pose_msg_to_homogeneous_matrix(pose_get_fk)
        try:
            pts,matrice_extr = localiser(type_plaque=,model_path=,image=img,M_hom_3D=Mtrix_hom_3D,M_pass_oc=,M_intr=,coeff_distr=)
        except TypeError:
            print("plaque non trouvée")


if __name__ == "__main__":
    rospy.init_node('test_localisation', anonymous=True)
    run_localisation()



