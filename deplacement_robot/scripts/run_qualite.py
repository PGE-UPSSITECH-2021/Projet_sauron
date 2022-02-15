#!/usr/bin/env python
from http.client import CONTINUE
from StepReader import StepReader
from useful_robot import rotation_between_vect, homogeneous_matrix_to_pose_msg
import numpy as np
import rospy
import rospkg
from deplacement_robot.srv import Robot_move, Robot_move_predef, Robot_set_state
from communication.srv import capture
from qualite_interface import fonction_qualite

from  geometry_msgs.msg import Pose
from deplacement_robot.msg import Qualite, Trou_qualite
from cv_bridge import CvBridge

def run_qualite(plaque_pos, nom_plaque, step_folder, dist =  0.18, diametres = [5,7,12,18], pub=None):
    pub_state(pub, "Debut conformite.")
    pub_state(pub, "Calcul de la trajectoire.")
    # Lecture du fichier step et recuperation de tous les trous
    d = get_holes(step_folder + "/" + str(nom_plaque) + ".stp", diametres)

    # Determination du chemin en fonction des trous
    points = get_path(plaque_pos, dist, d)

    pub_state(pub, "Trajectoire trouve.")

    ######## Deplacement du robot ########

    # Service pour deplacer le robot a un point donne
    move_robot = rospy.ServiceProxy('move_robot', Robot_move)
    # Service pour deplacer le robot a sa position de parking
    move_parking = rospy.ServiceProxy('move_robot_parking', Robot_move_predef)
    # Service pour prendre une image
    capture_image = rospy.ServiceProxy("camera/capture", capture)

    pub_state(pub, "Deplacement a la position de parking")
    move_parking()
    pub_state(pub, "Deplacement termine")

    bridge = CvBridge()
    returned_msg = Qualite()
    trous=[]

    nbPose = len(points)

    # Effectuer la trajectoire
    for i,p in enumerate(points):
        # On arrete si le node est kill
        if rospy.is_shutdown():
            exit()
        
        pub_state(pub, "Deplacement a la position " + str(i+1) + "/" + str(nbPose))
        # Le robot se deplace au point p
        resp1 = move_robot(p[0])

        pub_state(pub, "Deplacement termine")

        trou_qualite_msg = Trou_qualite()

        pub_state(pub, "Prise d'image")

        res = capture_image()

        pub_state(pub, "Verification de la conformite")

        rosimage = res.image
        cv_image = bridge.imgmsg_to_cv2(rosimage, 'bgr8')

        print("shape = ",cv_image.shape)
        assert (len(cv_image.shape) == 3),"(1) probleme dimensions, image BGR ?"

        isdefective, defect, image = fonction_qualite(p[1],cv_image,debug=False,fast_algo=True)


        image_ros_result = bridge.cv2_to_compressed_imgmsg(image)

        trou_qualite_msg.x= p[2][0]
        trou_qualite_msg.y= p[2][1]
        trou_qualite_msg.diam= p[1]
        trou_qualite_msg.conforme   = isdefective
        trou_qualite_msg.raison     = defect
        trou_qualite_msg.image      = image_ros_result

        trous.append(trou_qualite_msg)

        # Pour les tests
        #print(trou_qualite_msg)
        #print("press enter")
        #raw_input()

    #returned_msg.image=None #TODO
    returned_msg.trous = trous
    returned_msg.nbTrous = len(trous)

    if rospy.is_shutdown():
        exit()

    pub_state(pub, "Conformite finie, retour au parking")

    # Retour a la position de parking
    move_parking()

    pub_state(pub, "Conformite terminee.")

    return returned_msg


def pub_state(pub, msg):
    if not pub is None:
        pub.publish(msg)

def get_holes(file_path, diametres):
    step = StepReader(file_path)

    d = {}
    
    for c in step.getCylinders():
        if c.rayon <= 10 and c.rayon*2 in diametres:
            d[c.position] = c

    return d

def get_path(plaque_pos, dist, d):
    points = []
            
    keys = sorted(list(d.keys()))

    for k in keys :
        v = np.array(d[k].direction)
        m = np.array(k)/1000
        m = np.dot(plaque_pos, np.hstack((m, 1)))[:3]

        p1 = m + dist * v
        p2 = m - dist * v

        p = None

        if p2[2] > p1[2]:
            p = np.array([p2[:3]])
        else :
            v= -v
            p = np.array([p1[:3]])

        R = get_orientation_mat(v)

        h = np.hstack((R,p.T))
        h = np.vstack((h, [0,0,0,1]))

        rayon = d[k].rayon
        #pos=d[k].position
        msg = homogeneous_matrix_to_pose_msg(h)

        points.append((msg, rayon, m))

    return points

def get_orientation_mat(tz):
    tz = tz / np.linalg.norm(tz)
    tx = np.cross(tz,[0,1,0])
    tx = tx / np.linalg.norm(tx)
    ty = np.cross(tz, tx)
    ty = ty / np.linalg.norm(ty)

    R = np.hstack((np.array([tx]).T, np.array([ty]).T, np.array([tz]).T))

    return R

if __name__ == "__main__":
    rospy.init_node('test_identification', anonymous=True)
    R = np.eye(4)
    R[0,3] = 0.55
    R[1,3] = 0.24
    R[2,3] = -0.270 + 0.275
    rospack = rospkg.RosPack()
    cwd = rospack.get_path("deplacement_robot")
    run_qualite(R, "Plaque_1", cwd + "/plaques")
